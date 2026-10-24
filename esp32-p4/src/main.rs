#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ll::{MipiDsiPhyPllRefclkConfig, MipiDsiPhyPllRefclkSclk},
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    mipi_dsi::{
        dpi::{ColorFormat, DpiClockSource, DpiConfig, FrameTiming},
        Config as DsiConfig,
        DataLanes,
        MipiDsi,
    },
    psram::{Psram, PsramConfig},
    Config as HalConfig,
    main,
};

esp_bootloader_esp_idf::esp_app_desc!();

const W: usize = 480;
const H: usize = 800;
const FB_BYTES: usize = W * H * 2;

const INIT: &[(u8, &[u8], u32)] = &[
    (0xFF, &[0x77, 0x01, 0x00, 0x00, 0x13], 0),
    (0xEF, &[0x08], 0),
    (0xFF, &[0x77, 0x01, 0x00, 0x00, 0x10], 0),
    (0xC0, &[0x63, 0x00], 0),
    (0xC1, &[0x0D, 0x02], 0),
    (0xC2, &[0x17, 0x08], 0),
    (0xCC, &[0x10], 0),
    (0xB0, &[0x40, 0xC9, 0x94, 0x0E, 0x10, 0x05, 0x0B, 0x09, 0x08, 0x26, 0x04, 0x52, 0x10, 0x69, 0x6B, 0x69], 0),
    (0xB1, &[0x40, 0xD2, 0x98, 0x0C, 0x92, 0x07, 0x09, 0x08, 0x07, 0x25, 0x02, 0x0E, 0x0C, 0x6E, 0x78, 0x55], 0),
    (0xFF, &[0x77, 0x01, 0x00, 0x00, 0x11], 0),
    (0xB0, &[0x5D], 0),
    (0xB1, &[0x4E], 0),
    (0xB2, &[0x87], 0),
    (0xB3, &[0x80], 0),
    (0xB5, &[0x4E], 0),
    (0xB7, &[0x85], 0),
    (0xB8, &[0x21], 0),
    (0xB9, &[0x10, 0x1F], 0),
    (0xBB, &[0x03], 0),
    (0xBC, &[0x00], 0),
    (0xC1, &[0x78], 0),
    (0xC2, &[0x78], 0),
    (0xD0, &[0x88], 0),
    (0xE0, &[0x00, 0x3A, 0x02], 0),
    (0xE1, &[0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40], 0),
    (0xE2, &[0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00], 0),
    (0xE3, &[0x00, 0x00, 0x33, 0x33], 0),
    (0xE4, &[0x44, 0x44], 0),
    (0xE5, &[0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0], 0),
    (0xE6, &[0x00, 0x00, 0x33, 0x33], 0),
    (0xE7, &[0x44, 0x44], 0),
    (0xE8, &[0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06, 0x2B, 0xA0, 0xA0], 0),
    (0xEB, &[0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00], 0),
    (0xEC, &[0x08, 0x01], 0),
    (0xED, &[0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A, 0x89, 0xB2, 0x0B], 0),
    (0xEF, &[0x08, 0x08, 0x08, 0x45, 0x3F, 0x54], 0),
    (0xFF, &[0x77, 0x01, 0x00, 0x00, 0x00], 0),
    (0x11, &[], 120),
    (0x29, &[], 0),
];

fn draw_pattern(fb: &mut [u8], tick: u32) {
    let mut i = 0;
    while i < fb.len() {
        let p = i / 2;
        let x = p % W;
        let y = p / W;
        let c = if (x ^ (y as usize / 32)) % 2 == 0 { 0 } else { 0xFFFF };
        fb[i] = (c & 0xFF) as u8;
        fb[i + 1] = (c >> 8) as u8;
        i += 2;
    }
    let bar_base = ((tick as usize * 4) % (H - 64)) * W;
    let mut b = bar_base * 2;
    while b < b.wrapping_add(64 * W * 2) {
        fb[b] = 0x00;
        fb[b + 1] = 0x7B;
        b += 2;
    }
}

#[main]
fn main() -> ! {
    esp_println::println!("thermo-esp32-p4 starting");
    let peripherals = esp_hal::init(HalConfig::default());
    let delay = Delay::new();

    let psram = Psram::new(peripherals.PSRAM, PsramConfig::default());
    let (fb_base, fb_len) = psram.raw_parts();
    esp_println::println!("psram: base={:#010x} len={}", fb_base as usize, fb_len);
    assert!(fb_len >= FB_BYTES, "psram too small");

    let mut bl = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let mut rst = Output::new(peripherals.GPIO27, Level::High, OutputConfig::default());
    rst.set_high();
    delay.delay_millis(10);
    rst.set_low();
    delay.delay_millis(20);
    rst.set_high();
    delay.delay_millis(150);
    bl.set_low();

    let dsi = MipiDsi::new(
        peripherals.MIPI_DSI,
        peripherals.VDMA_CH0,
        DsiConfig::default()
            .with_num_data_lanes(DataLanes::_2)
            .with_lane_bit_rate_mbps(500.0)
            .with_phy_pll_refclk(MipiDsiPhyPllRefclkConfig::new(
                MipiDsiPhyPllRefclkSclk::Xtal,
                0,
            ))
            .with_force_clock_lane_hs(false),
    )
    .expect("dsi bus");

    esp_println::println!("dsi bus up");
    let mut dsi = dsi;
    {
        let mut dbi = dsi.dbi(0);
        for &(cmd, params, ms) in INIT {
            dbi.write_cmd(cmd, params).expect("dbi write");
            if ms > 0 {
                delay.delay_millis(ms);
            }
        }
    }
    esp_println::println!("panel init sent");

    let fb: &'static mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(fb_base, FB_BYTES) };

    let mut dpi = dsi
        .dpi(
            DpiConfig {
                virtual_channel: 0,
                pixel_clock_mhz: 30.0,
                dpi_clk_src: DpiClockSource::Xtal,
                in_color_format: ColorFormat::Rgb565,
                out_color_format: ColorFormat::Rgb565,
                timing: FrameTiming {
                    h_active: W as u32,
                    hsw: 12,
                    hbp: 42,
                    hfp: 42,
                    v_active: H as u32,
                    vsw: 8,
                    vbp: 2,
                    vfp: 60,
                },
            },
            &[fb],
        )
        .expect("dpi start");
    esp_println::println!("dpi streaming");

    let mut tick = 0u32;
    loop {
        draw_pattern(dpi.framebuffer_mut(), tick);
        dpi.commit();
        dpi.wait_for_vsync();
        tick = tick.wrapping_add(1);
    }
}