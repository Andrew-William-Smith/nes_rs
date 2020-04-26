use super::cpu;
use glium::glutin::dpi::LogicalSize;
use glium::glutin::event::{Event, WindowEvent};
use glium::glutin::event_loop::{ControlFlow, EventLoop};
use glium::glutin::window::WindowBuilder;
use glium::glutin::ContextBuilder;
use glium::{Display, Surface};
use imgui::{Context, Ui};
use imgui_glium_renderer::Renderer;
use imgui_winit_support::{HiDpiMode, WinitPlatform};
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Factor into which to divide the hi-DPI scale factor for font scaling.
const HI_DPI_NUMERATOR: f32 = 2.0;

/// Grayscale colour used for the window background.
const BACKGROUND_COLOUR: f32 = 0.15;
/// Orange colour used to highlight UI elements.
pub const HIGHLIGHT_COLOUR: [f32; 4] = [1.0, 0.65, 0.0, 1.0];

/// A single instance of the `nes_rs` user interface.
pub struct NesUi {
    event_loop: EventLoop<()>,
    display: Display,
    imgui: Context,
    platform: WinitPlatform,
    renderer: Renderer,
}

impl NesUi {
    /// Instantiate a new `nes_rs` user interface window.  Should only be called once.
    fn new() -> NesUi {
        // Initialise a display in a new window
        let event_loop = EventLoop::new();
        let context = ContextBuilder::new().with_vsync(true);
        let builder = WindowBuilder::new()
            .with_title("nes_rs")
            .with_inner_size(LogicalSize::new(1024f64, 768f64));
        let display =
            Display::new(builder, context, &event_loop).expect("Failed to initialise display!");

        // Initialise ImGui context and window
        let mut imgui = Context::create();
        let mut platform = WinitPlatform::init(&mut imgui);
        {
            let gl_window = display.gl_window();
            platform.attach_window(imgui.io_mut(), &gl_window.window(), HiDpiMode::Rounded);
        }
        // Set ImGui font size to be readable on high-DPI displays
        imgui.io_mut().font_global_scale = HI_DPI_NUMERATOR / platform.hidpi_factor() as f32;

        // Initialise renderer
        let renderer =
            Renderer::init(&mut imgui, &display).expect("Failed to initialise renderer!");

        NesUi {
            event_loop,
            display,
            imgui,
            platform,
            renderer,
        }
    }

    /// Initialise the `nes_ui` main loop.  Derived from `imgui-rs` examples.
    /// # Arguments
    /// - `run_ui` - The function to run inside the main loop.
    fn main_loop<F: FnMut(&mut bool, &mut Ui) + 'static>(self, mut run_ui: F) {
        // Unpack for lifetime maintenance
        let NesUi {
            event_loop,
            display,
            mut imgui,
            mut platform,
            mut renderer,
            ..
        } = self;

        let mut frame_time = Instant::now();
        event_loop.run(move |event, _, control_flow| match event {
            // Set time at which this frame started
            Event::NewEvents(_) => frame_time = imgui.io_mut().update_delta_time(frame_time),
            // Handle new window
            Event::MainEventsCleared => {
                let gl_window = display.gl_window();
                platform
                    .prepare_frame(imgui.io_mut(), &gl_window.window())
                    .expect("Failed to prepare ImGui frame!");
                gl_window.window().request_redraw();
            }
            // Redraw request sent
            Event::RedrawRequested(_) => {
                // Display a frame of the main UI
                let mut ui = imgui.frame();
                let mut continue_run = true;
                run_ui(&mut continue_run, &mut ui);
                // Handle UI exit
                if !continue_run {
                    *control_flow = ControlFlow::Exit;
                }

                // Fill the window background with dark gray
                let gl_window = display.gl_window();
                let mut frame = display.draw();
                frame.clear_color_srgb(
                    BACKGROUND_COLOUR,
                    BACKGROUND_COLOUR,
                    BACKGROUND_COLOUR,
                    1.0,
                );
                platform.prepare_render(&ui, gl_window.window());
                renderer
                    .render(&mut frame, ui.render())
                    .expect("Fame rendering failed!");
                frame.finish().expect("Failed to swap frame buffers!");
            }
            // Window close
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => *control_flow = ControlFlow::Exit,
            // Pass all other results on to platform handler
            event => {
                let gl_window = display.gl_window();
                platform.handle_event(imgui.io_mut(), gl_window.window(), &event);
            }
        });
    }

    /// Run the main loop for this UI instance.  Does not terminate.
    pub fn run_loop(cpu: Arc<Mutex<cpu::CPU>>) {
        let interface = NesUi::new();
        // Begin UI main loop
        interface.main_loop(move |_, ui| {
            cpu.lock().unwrap().display(ui);
        });
    }
}

/// A component that provides a visualisation in the UI.
pub trait Visualisable {
    /// Render the component's visualisation in the specified ImGui interface.
    fn display(&mut self, ui: &Ui);
}
