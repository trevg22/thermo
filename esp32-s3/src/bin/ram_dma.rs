//! Diagnoses whether DMA-from-PSRAM is the cause of partial-frame "drift".
//! Uses a half-width (400x480) framebuffer placed in internal SRAM and streams
//! it repeatedly. The panel's left half should show all 10 bands.

#![no_std]
#![no_main]

extern crate alloc;

use core::slice;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{aligned::DmaAlignedMut, DmaTxBuf},
    gpio::Level,
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

const WIDTH: usize = 240;
const HEIGHT: usize = 480;
const FRAME_BYTES: usize = WIDTH * HEIGHT * 2;
const DMA_CHUNK: usize = 4032;

const CH422G_WR_SET: u8 = 0x24;
const CH422G_WR_OC: u8 = 0x23;
const CH422G_WR_IO: u8 = 0x38;
const EXP_LCD_RST: u8 = 1 << 3;

#[repr(align(64))]
struct AlignedFb([u8; FRAME_BYTES]);

static mut FB: AlignedFb = AlignedFb([0u8; FRAME_BYTES]);

fn diag_config() -> DpiConfig {
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
            horizontal_active_width: 240,
            horizontal_total_width: 260,
            horizontal_blank_front_porch: 6,
            vertical_active_height: 480,
            vertical_total_height: 500,
            vertical_blank_front_porch: 12,
            hsync_width: 4,
            vsync_width: 4,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false)
}

fn paint_pattern(buf: &mut [u8]) {
    for y in 0..HEIGHT {
        let c: u16 = if y < 336 {
            0xF800
        } else {
            // 8-row alternating stripes from row 336 to 479: locate exact cutoff.
            if ((y - 336) / 8) % 2 == 0 {
                0xFFFF
            } else {
                0x0000
            }
        };
        for x in 0..WIDTH {
            let px = (y * WIDTH + x) * 2;
            buf[px] = c as u8;
            buf[px + 1] = (c >> 8) as u8;
        }
    }
}

fn dump_regs() {
    const BASE: usize = 0x6004_1000;
    for i in 0..16 {
        let v = unsafe { core::ptr::read_volatile((BASE + i * 4) as *const u32) };
        println!("LCD_CAM[0x{:02x}] = 0x{:08x}", i * 4, v);
    }
}

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

    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let dpi = Dpi::new(lcd_cam.lcd, tx_channel, diag_config())
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

    // Internal SRAM framebuffer (static, 64-byte aligned).
    let fb_ptr = core::ptr::addr_of_mut!(FB).cast::<u8>();
    let fb: &'static mut [u8] = unsafe { slice::from_raw_parts_mut(fb_ptr, FRAME_BYTES) };
    paint_pattern(fb);

    let (_, tx_desc) = esp_hal::dma_descriptors_chunk_size!(0, FRAME_BYTES, DMA_CHUNK);
    let mut dma_buf = DmaTxBuf::new(
        unsafe { DmaAlignedMut::new_unchecked(tx_desc) },
        unsafe { DmaAlignedMut::new_unchecked(fb) },
    )
    .expect("tx buf");

    let mut dpi = dpi;
    println!("ram dma loop: 240x480 from internal SRAM, ruler pattern @16MHz");
    dump_regs();
    println!("regs dumped");

    let mut frame = 0u32;
    let mut last = esp_hal::time::Instant::now();
    loop {
        let transfer = dpi.send(false, dma_buf).map_err(|(e, _, _)| e).expect("send");
        (_, dpi, dma_buf) = transfer.wait();
        frame += 1;
        if frame > 120 {
            let dt = (esp_hal::time::Instant::now() - last).as_micros();
            println!("120 frames in {}us -> {:.1} fps", dt, 120_000_000.0 / dt as f32);
            dump_regs();
            println!("regs dumped");
            frame = 0;
            last = esp_hal::time::Instant::now();
        }
    }
}

esp_bootloader_esp_idf::esp_app_desc!();