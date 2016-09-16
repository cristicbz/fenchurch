use gfx::{Window, WindowOptions, Camera, Input, Gesture, Scancode};
use math::Vec3f;
use super::controller::{Controller, ControllerBindings};
use super::errors::{Result, ChainErr};
use super::frame_timers::{FrameTimers, FrameTimerId};
use super::sphere_renderer::{SphereRenderer, SphereList};
use super::simulation::{Simulation, NewEntity};
use rand::{self, Rng};

pub struct App {
    window: Window,
    input: Input,
    renderer: SphereRenderer,
    camera: Camera,
    controller: Controller,
    timers: FrameTimers,

    simulation: Simulation,

    frame_timer: FrameTimerId,
    cpu_timer: FrameTimerId,
    render_timer: FrameTimerId,
    sim_timer: FrameTimerId,
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
        let mut camera = Camera::new(75.0, window.aspect_ratio(), 0.1, 100.0);
        let mut timers = FrameTimers::new();
        camera.set_position(Vec3f::new(5.9232, 6.990, 15.044));
        camera.set_yaw(-0.5519);
        camera.set_pitch(0.2919);

        let simulation = Simulation::with_capacity(&mut timers, 16384);

        Ok(App {
            window: window,
            renderer: renderer,
            input: input,
            camera: camera,
            controller: Controller::new(ControllerBindings::default()),

            frame_timer: timers.new_stopped("frame"),
            cpu_timer: timers.new_stopped("cpu"),
            render_timer: timers.new_stopped("render"),
            sim_timer: timers.new_stopped("sim"),
            timers: timers,

            simulation: simulation,
        })
    }

    pub fn run(mut self) -> Result<()> {
        let quit_gesture = Gesture::AnyOf(vec![Gesture::QuitTrigger,
                                               Gesture::KeyTrigger(Scancode::Escape)]);
        let explode_gesture = Gesture::KeyHold(Scancode::E);

        let num_spheres = 30000;
        let mut rng = rand::ChaChaRng::new_unseeded();
        for _ in 0..num_spheres {
            let position = Vec3f::new((rng.gen::<f32>() - 0.5) * 2.0 * 5.0,
                                      (rng.gen::<f32>() - 0.5) * 2.0 * 5.0 + 10.0,
                                      (rng.gen::<f32>() - 0.5) * 2.0 * 5.0);
            let velocity = Vec3f::new((rng.gen::<f32>() - 0.5) * 10. + 10.0,
                                      (rng.gen::<f32>() - 0.5) * 10. - 5.0,
                                      (rng.gen::<f32>() - 0.5) * 10.0);
            self.simulation.add(NewEntity {
                position: position,
                velocity: velocity,
                mass: 1.0,
                radius: 0.1,
                colour: Vec3f::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>()) * 0.7 +
                        Vec3f::new(0.2, 0.2, 0.2),
            });
        }

        info!("Entering main loop...");
        let mut running = true;
        while running {
            let mut frame = self.window.draw();
            let frame_result = (|| -> Result<()> {
                let delta_time = self.timers.start(self.frame_timer).unwrap_or(1.0 / 60.0);
                self.timers.start(self.cpu_timer);
                self.input.update();

                self.timers.start(self.render_timer);
                try!(self.renderer
                    .render(&self.window,
                            &self.camera,
                            &mut frame,
                            SphereList {
                                positions: self.simulation.positions(),
                                radii: self.simulation.radii(),
                                colours: self.simulation.colours(),
                            })
                    .chain_err(|| "Failed to render frame."));
                self.timers.stop(self.render_timer);

                if self.input.poll_gesture(&quit_gesture) {
                    running = false;
                }
                if self.input.poll_gesture(&explode_gesture) {
                    self.simulation.explode();
                }
                self.controller.update(delta_time, &mut self.input, &mut self.camera);

                self.timers.start(self.sim_timer);
                self.simulation.update(&mut self.timers, delta_time);
                self.timers.stop(self.sim_timer);

                self.timers.stop(self.cpu_timer);
                Ok(())
            })();
            try!(frame.finish().chain_err(|| "Context lost."));
            try!(frame_result);
        }

        Ok(())
    }
}
