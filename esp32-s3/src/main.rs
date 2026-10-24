#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::slice;
use core::time::Duration;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    dma::{
        aligned::DmaAlignedMut,
        BurstConfig, DmaDescriptor, DmaTxBuffer, Owner, Preparation,
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
    time::{Instant, Rate},
};
use slint::{
    platform::{
        software_renderer::{
            MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel, SoftwareRenderer,
        },
        Platform, PlatformError, PointerEventButton, WindowAdapter, WindowEvent,
    },
    LogicalPosition, PhysicalSize,
};

slint::include_modules!();

mod gt911;

// The ST7262 RGB panel is driven purely by DPI signals, no init sequence.
const WIDTH: usize = 800;
const HEIGHT: usize = 480;
const FRAME_BYTES: usize = WIDTH * HEIGHT * 2; // RGB565
const FRAME_ALIGN: usize = 64; // PSRAM cache line / DMA alignment
const DMA_CHUNK: usize = 4032; // multiple of 64, <= 4095

// CH422G IO expander: every "register" is its own 7-bit I2C address.
const CH422G_WR_SET: u8 = 0x24; // 0x48 >> 1 (system config: IO_OE)
const CH422G_WR_OC: u8 = 0x23; // 0x46 >> 1 (open-drain outputs)
const CH422G_WR_IO: u8 = 0x38; // 0x70 >> 1 (push-pull outputs)

const EXP_TP_RST: u8 = 1 << 1;
const EXP_LCD_RST: u8 = 1 << 3;

struct GuiPlatform {
    window: Rc<MinimalSoftwareWindow>,
    start: Instant,
}

impl Platform for GuiPlatform {
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> Duration {
        Duration::from_micros(self.start.elapsed().as_micros())
    }
}

// A self-looping DMA descriptor chain over the PSRAM framebuffer. The LCD
// consumes one frame (800x480x2 bytes) per screen refresh and the chain wraps
// back to the start, so the panel keeps refreshing forever without the CPU
// having to re-arm the DMA. Data is rendered into the framebuffer and the
// PSRAM cache written back whenever Slint redraws.
struct CircularFrameBuf {
    descriptors: DmaAlignedMut<'static, [DmaDescriptor]>,
}

unsafe impl DmaTxBuffer for CircularFrameBuf {
    type View = CircularFrameBuf;
    type Final = CircularFrameBuf;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.descriptors.as_mut_ptr(),
            #[cfg(dma_can_access_psram)]
            accesses_psram: true,
            burst_transfer: BurstConfig::default(),
            // The DMA must not check the owner bit, it is never updated.
            check_owner: Some(false),
            auto_write_back: false,
        }
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self {
        view
    }
}

fn make_frame_buf(descriptors: &'static mut [DmaDescriptor], fb_addr: usize) -> CircularFrameBuf {
    let mut descriptors = unsafe { DmaAlignedMut::new_unchecked(descriptors) };
    let ndesc = descriptors.len();
    let base = descriptors.as_mut_ptr();
    for (i, d) in descriptors.iter_mut().enumerate() {
        let len = if i == ndesc - 1 {
            FRAME_BYTES - i * DMA_CHUNK
        } else {
            DMA_CHUNK
        };
        d.set_owner(Owner::Dma);
        d.set_suc_eof(false);
        d.set_length(len);
        d.set_size(len);
        d.buffer = (fb_addr + i * DMA_CHUNK) as *mut u8;
        d.next = if i + 1 < ndesc {
            unsafe { base.add(i + 1) }
        } else {
            base
        };
    }
    CircularFrameBuf { descriptors }
}

fn render_frame(renderer: &SoftwareRenderer, fb_addr: usize) {
    let pixels: &mut [Rgb565Pixel] =
        unsafe { slice::from_raw_parts_mut(fb_addr as *mut Rgb565Pixel, WIDTH * HEIGHT) };
    renderer.render(pixels, WIDTH);

    // Flush the rendered PSRAM framebuffer to RAM so the DMA sees it.
    let bytes: &mut [u8] = unsafe { slice::from_raw_parts_mut(fb_addr as *mut u8, FRAME_BYTES) };
    let mut aligned = unsafe { DmaAlignedMut::new_unchecked(bytes) };
    aligned.writeback();
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

    // --- I2C bus shared by the CH422G expander and the GT911 touch controller
    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO8)
    .with_scl(peripherals.GPIO9);

    // --- CH422G init: system enable, open-drain outputs on, all lines high.
    // LCD backlight is expander bit 2, so this also lights the panel.
    i2c.write(CH422G_WR_SET, &[0x01]).expect("ch422g set");
    i2c.write(CH422G_WR_OC, &[0x0F]).expect("ch422g oc");
    i2c.write(CH422G_WR_IO, &[0xFF]).expect("ch422g io");

    // --- LCD reset pulse (active low) on expander bit 3
    i2c.write(CH422G_WR_IO, &[0xFF & !EXP_LCD_RST]).expect("lcd rst low");
    delay.delay_millis(10);
    i2c.write(CH422G_WR_IO, &[0xFF]).expect("lcd rst high");
    delay.delay_millis(100);

    // --- GT911 reset: pull INT low while TP_RST (expander bit 1) pulses.
    let mut tp_int = peripherals.GPIO4;
    let int_drive = Output::new(tp_int.reborrow(), Level::Low, OutputConfig::default());
    delay.delay_millis(10);
    i2c.write(CH422G_WR_IO, &[0xFF & !EXP_TP_RST]).expect("tp rst low");
    delay.delay_millis(100);
    i2c.write(CH422G_WR_IO, &[0xFF]).expect("tp rst high");
    delay.delay_millis(200);
    drop(int_drive);
    let _tp_int_input = Input::new(tp_int, InputConfig::default().with_pull(Pull::None));

    // --- Touch (optional; the GT911 may not be reachable until the panel is up)
    let mut touch = gt911::Gt911::new(i2c);

    // --- RGB/DPI interface with the Waveshare reference timings (16 MHz)
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let config = DpiConfig::default()
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
            horizontal_blank_front_porch: 12, // hsync_width (4) + HBP (8)
            vertical_active_height: 480,
            vertical_total_height: 500,
            vertical_blank_front_porch: 12, // vsync_width (4) + VBP (8)
            hsync_width: 4,
            vsync_width: 4,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
        .expect("dpi config")
        .with_vsync(peripherals.GPIO3)
        .with_hsync(peripherals.GPIO46)
        .with_de(peripherals.GPIO5)
        .with_pclk(peripherals.GPIO7)
        // Blue bits
        .with_data0(peripherals.GPIO14)
        .with_data1(peripherals.GPIO38)
        .with_data2(peripherals.GPIO18)
        .with_data3(peripherals.GPIO17)
        .with_data4(peripherals.GPIO10)
        // Green bits
        .with_data5(peripherals.GPIO39)
        .with_data6(peripherals.GPIO0)
        .with_data7(peripherals.GPIO45)
        .with_data8(peripherals.GPIO48)
        .with_data9(peripherals.GPIO47)
        .with_data10(peripherals.GPIO21)
        // Red bits
        .with_data11(peripherals.GPIO1)
        .with_data12(peripherals.GPIO2)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO41)
        .with_data15(peripherals.GPIO40);

    // --- Framebuffer in PSRAM (DMA-accessible, 64-byte aligned)
    let fb_layout = core::alloc::Layout::from_size_align(FRAME_BYTES, FRAME_ALIGN).expect("fb layout");
    let fb_addr = unsafe { alloc::alloc::alloc_zeroed(fb_layout) } as usize;
    assert!(fb_addr != 0, "framebuffer allocation failed");

    // --- Slint platform
    let window = MinimalSoftwareWindow::new(RepaintBufferType::SwappedBuffers);
    slint::platform::set_platform(Box::new(GuiPlatform {
        window: window.clone(),
        start: Instant::now(),
    }))
    .expect("failed to install platform");

    window.set_size(PhysicalSize::new(WIDTH as u32, HEIGHT as u32));
    let _ui = MainWindow::new();

    // --- Render the first frame, then start the self-looping DMA.
    let _ = window.draw_if_needed(|renderer| render_frame(renderer, fb_addr));

    let (_, raw_descriptors) = esp_hal::dma_descriptors_chunk_size!(0, FRAME_BYTES, DMA_CHUNK);
    let mut frame_buf = make_frame_buf(raw_descriptors, fb_addr);
    // Make the freshly-written descriptor chain visible to the DMA.
    frame_buf.descriptors.writeback();

    let _transfer = dpi
        .send(true, frame_buf)
        .map_err(|(e, _, _)| e)
        .expect("dpi send");

    // --- Event loop
    let mut touch_down = false;
    let mut last_pos = LogicalPosition::new(0.0, 0.0);
    let mut touch_points = [gt911::TouchPoint { x: 0, y: 0 }; 5];

    loop {
        slint::platform::update_timers_and_animations();

        if let Some(gt) = touch.as_mut() {
            let n = gt.read(&mut touch_points);
            if n > 0 {
                let pos = LogicalPosition::new(
                    (touch_points[0].x as f32).clamp(0.0, WIDTH as f32),
                    (touch_points[0].y as f32).clamp(0.0, HEIGHT as f32),
                );
                if touch_down {
                    window
                        .window()
                        .dispatch_event(WindowEvent::PointerMoved { position: pos });
                } else {
                    window.window().dispatch_event(WindowEvent::PointerPressed {
                        position: pos,
                        button: PointerEventButton::Left,
                    });
                    touch_down = true;
                }
                last_pos = pos;
            } else if touch_down {
                window
                    .window()
                    .dispatch_event(WindowEvent::PointerReleased {
                        position: last_pos,
                        button: PointerEventButton::Left,
                    });
                touch_down = false;
            }
        }

        let _redrawn = window.draw_if_needed(|renderer| render_frame(renderer, fb_addr));

        delay.delay_millis(5);
    }
}

esp_bootloader_esp_idf::esp_app_desc!();