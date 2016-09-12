use gfx::{Window, WindowOptions, Camera, Input, Gesture, Scancode};
use math::Vec3f;
use super::controller::{Controller, ControllerBindings};
use super::errors::{Result, ChainErr};
use super::frame_timers::{FrameTimers, FrameTimerId};
use num::Zero;
use super::sphere_renderer::{SphereRenderer, SphereList};

pub struct App {
    window: Window,
    input: Input,
    renderer: SphereRenderer,
    camera: Camera,
    controller: Controller,
    timers: FrameTimers,
    frame_timer: FrameTimerId,
    cpu_timer: FrameTimerId,
}

impl App {
    pub fn new() -> Result<Self> {
        let window = try!(Window::new(WindowOptions {
                title: "Fenchurch v0.1".into(),
                width: 1920,
                height: 1080,
                background: (0.06, 0.07, 0.09, 0.0),
                ..Default::default()
            })
            .chain_err(|| "Failed to create window,"));
        let renderer = try!(SphereRenderer::new(&window, Default::default())
            .chain_err(|| "Failed to create renderer."));
        let input = try!(Input::new(&window).chain_err(|| "Failed to create input."));
        let mut camera = Camera::new(75.0, window.aspect_ratio(), 0.1, 1000.0);
        let mut timers = FrameTimers::new();
        camera.set_position(Vec3f::new(0.0, 0.0, 5.0));

        let frame_timer = timers.new_stopped("frame");
        let cpu_timer = timers.new_stopped("cpu");

        Ok(App {
            window: window,
            renderer: renderer,
            input: input,
            camera: camera,
            controller: Controller::new(ControllerBindings::default()),
            timers: timers,
            frame_timer: frame_timer,
            cpu_timer: cpu_timer,
        })
    }

    pub fn run(mut self) -> Result<()> {
        let quit_gesture = Gesture::AnyOf(vec![Gesture::QuitTrigger,
                                               Gesture::KeyTrigger(Scancode::Escape)]);

        let num_spheres = 20000;
        let mut positions = vec![Vec3f::zero(); num_spheres];
        let mut radii = vec![0.0f32; num_spheres];
        let mut colours = vec![Vec3f::zero(); num_spheres];

        info!("Entering main loop...");
        let mut running = true;
        let mut time = 100.0;
        while running {
            let mut frame = self.window.draw();
            let frame_result = (|| -> Result<()> {
                let delta_time = self.timers.start(self.frame_timer).unwrap_or(1.0 / 60.0);
                self.timers.start(self.cpu_timer);
                self.input.update();

                time += delta_time;
                for (i_sphere, (position, (radius, colour))) in positions.iter_mut()
                    .zip(radii.iter_mut().zip(colours.iter_mut()))
                    .enumerate() {
                    let phase = i_sphere as f32 * 0.01 + time;
                    let frequency = ((i_sphere as f32 * 0.01).sin() + 2.0).ln() *
                                    ((time * 0.5).sin() * 0.1 + 1.0);
                    let base = (phase * frequency).sin();

                    let pos_angle = i_sphere as f32 * 0.01;
                    let distance = pos_angle.powf(0.9) + 0.01;
                    *position = Vec3f::new((pos_angle + phase / distance).sin() * distance,
                                           base * 5.0 * frequency,
                                           (pos_angle + phase / distance).cos() * distance);
                    *radius = (base + 2.0) / 2.0 * 0.5;
                    *colour = Vec3f::new((phase * frequency * 0.5).sin() * 0.5 + 0.5,
                                         ((phase + 0.1) * frequency * 1.1).sin() * 0.5 + 0.5,
                                         ((phase + 0.2) * frequency * 0.9).sin() * 0.5 + 0.5);
                }

                try!(self.renderer
                    .render(&self.window,
                            &self.camera,
                            &mut frame,
                            SphereList {
                                positions: &positions,
                                radii: &radii,
                                colours: &colours,
                            })
                    .chain_err(|| "Failed to render frame."));
                if self.input.poll_gesture(&quit_gesture) {
                    running = false;
                }
                self.controller.update(delta_time, &mut self.input, &mut self.camera);
                self.timers.stop(self.cpu_timer);
                Ok(())
            })();
            try!(frame.finish().chain_err(|| "Context lost."));
            try!(frame_result);
        }

        Ok(())
    }
}
