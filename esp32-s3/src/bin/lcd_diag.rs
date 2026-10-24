//! Display diagnostics: sweeps clock mode / byte order / sync polarity and
//! prints each combo over USB-serial-JTAG so a working combination can be
//! identified on-screen.

#![no_std]
#![no_main]

extern crate alloc;

#[path = "../gt911.rs"]
mod gt911;

use core::slice;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{
        aligned::DmaAlignedMut,
        DmaTxBuf,
    },
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    lcd_cam::{
        LcdCam,
        lcd::{
            ClockMode, Phase, Polarity,
            dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
        },
    },
    main,
    peripherals::Peripherals,
    time::Rate,
};
use esp_println::println;

const WIDTH: usize = 800;
const HEIGHT: usize = 480;
const FRAME_BYTES: usize = WIDTH * HEIGHT * 2;
const FRAME_ALIGN: usize = 64;
const DMA_CHUNK: usize = 4032;

const CH422G_WR_SET: u8 = 0x24;
const CH422G_WR_OC: u8 = 0x23;
const CH422G_WR_IO: u8 = 0x38;
const EXP_TP_RST: u8 = 1 << 1;
const EXP_LCD_RST: u8 = 1 << 3;

#[derive(Clone, Copy, Debug)]
struct Combo {
    de_idle: Level,
    no_black_region: bool,
    marker: u16,
}

const COMBOS: &[Combo] = &[
    Combo { de_idle: Level::High, no_black_region: false, marker: 0xF800 },
    Combo { de_idle: Level::High, no_black_region: true, marker: 0x07E0 },
    Combo { de_idle: Level::Low, no_black_region: false, marker: 0x001F },
    Combo { de_idle: Level::Low, no_black_region: true, marker: 0xFFE0 },
];

fn diag_config(c: &Combo) -> DpiConfig {
    DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(16))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: 800,
            horizontal_total_width: 820,
            horizontal_blank_front_porch: 12,
            vertical_active_height: 480,
            vertical_total_height: 500,
            vertical_blank_front_porch: 12,
            hsync_width: 4,
            vsync_width: 4,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(c.de_idle)
        .with_disable_black_region(c.no_black_region)
}

fn paint_pattern(buf: &mut [u8], marker: u16) {
    const BANDS: [u16; 10] = [
        0xF800, 0x07E0, 0x001F, 0xFFFF, 0x0000, 0xFFE0, 0xF81F, 0x07FF, 0x39E7, 0xF8A0,
    ];
    for y in 0..HEIGHT {
        let c = if y < 48 {
            marker
        } else {
            BANDS[(y / 48).min(9)]
        };
        for x in 0..WIDTH {
            let px = (y * WIDTH + x) * 2;
            buf[px] = c as u8;
            buf[px + 1] = (c >> 8) as u8;
        }
    }
}

const PIXELS_PER_FRAME: u32 = 820 * 500;

#[main]
fn main() -> ! {
    let peripherals: Peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));

    esp_alloc::psram_allocator!(
        peripherals.PSRAM,
        esp_hal::psram,
        esp_hal::psram::PsramConfig::default()
    );

    let delay = Delay::new();

    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO8)
    .with_scl(peripherals.GPIO9);

    i2c.write(CH422G_WR_SET, &[0x01]).expect("ch422g set");
    i2c.write(CH422G_WR_OC, &[0x0F]).expect("ch422g oc");
    i2c.write(CH422G_WR_IO, &[0xFF]).expect("ch422g io");

    i2c.write(CH422G_WR_IO, &[0xFF & !EXP_LCD_RST]).expect("lcd rst low");
    delay.delay_millis(10);
    i2c.write(CH422G_WR_IO, &[0xFF]).expect("lcd rst high");
    delay.delay_millis(100);

    let mut tp_int = peripherals.GPIO4;
    let int_drive = Output::new(tp_int.reborrow(), Level::Low, OutputConfig::default());
    delay.delay_millis(10);
    i2c.write(CH422G_WR_IO, &[0xFF & !EXP_TP_RST]).expect("tp rst low");
    delay.delay_millis(100);
    i2c.write(CH422G_WR_IO, &[0xFF]).expect("tp rst high");
    delay.delay_millis(200);
    drop(int_drive);
    let _tp_int_input = Input::new(tp_int, InputConfig::default().with_pull(Pull::None));
    let _touch = gt911::Gt911::new(i2c);

    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let dpi = Dpi::new(lcd_cam.lcd, tx_channel, diag_config(&COMBOS[0]))
        .expect("dpi config")
        .with_vsync(peripherals.GPIO3)
        .with_hsync(peripherals.GPIO46)
        .with_de(peripherals.GPIO5)
        .with_pclk(peripherals.GPIO7)
        .with_data0(peripherals.GPIO14)
        .with_data1(peripherals.GPIO38)
        .with_data2(peripherals.GPIO18)
        .with_data3(peripherals.GPIO17)
        .with_data4(peripherals.GPIO10)
        .with_data5(peripherals.GPIO39)
        .with_data6(peripherals.GPIO0)
        .with_data7(peripherals.GPIO45)
        .with_data8(peripherals.GPIO48)
        .with_data9(peripherals.GPIO47)
        .with_data10(peripherals.GPIO21)
        .with_data11(peripherals.GPIO1)
        .with_data12(peripherals.GPIO2)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO41)
        .with_data15(peripherals.GPIO40);

    let fb_layout = core::alloc::Layout::from_size_align(FRAME_BYTES, FRAME_ALIGN).expect("fb layout");
    let fb_ptr = unsafe { alloc::alloc::alloc_zeroed(fb_layout) } as *mut u8;
    assert!(!fb_ptr.is_null(), "framebuffer allocation failed");

    let fb: &'static mut [u8] = unsafe { slice::from_raw_parts_mut(fb_ptr, FRAME_BYTES) };
    let (_, tx_desc) = esp_hal::dma_descriptors_chunk_size!(0, FRAME_BYTES, DMA_CHUNK);
    let dma_buf = DmaTxBuf::new(
        unsafe { DmaAlignedMut::new_unchecked(tx_desc) },
        unsafe { DmaAlignedMut::new_unchecked(fb) },
    )
    .expect("tx buf");

    let mut combo_idx = 0usize;
    let mut frames_since_combo = 0u32;
    let mut dma_buf = dma_buf;
    let mut dpi = dpi;

    println!("diag de-black sweep start");

    loop {
        if frames_since_combo == 0 {
            let combo = COMBOS[combo_idx];
            dpi.apply_config(&diag_config(&combo)).expect("apply config");
            paint_pattern(dma_buf.as_mut_slice(), combo.marker);
            println!("phase {}: {:?}", combo_idx, combo);
            combo_idx = (combo_idx + 1) % COMBOS.len();
        }

        // Drive frames back-to-back so the pattern stays on screen.
        let transfer = dpi.send(false, dma_buf).map_err(|(e, _, _)| e).expect("send");
        (_, dpi, dma_buf) = transfer.wait();

        frames_since_combo += 1;
        if frames_since_combo >= 480 {
            frames_since_combo = 0;
        }
    }
}

esp_bootloader_esp_idf::esp_app_desc!();