use gfx::{Window, WindowOptions, Camera, Input, Gesture, Scancode};
use math::{Vec3f, Mat4};
use rand::{self, Rng};
use super::controller::{Controller, ControllerBindings};
use super::errors::{Result, ChainErr};
use super::frame_timers::{FrameTimers, FrameTimerId};
use super::mesh_renderer::{MeshRenderer, MeshId, MeshList};
use super::simulation::{Simulation, NewEntity};
use super::sphere_renderer::{SphereRenderer, SphereList};
use super::heightmap::Heightmap;
use num::Zero;
use std::time::Instant;


pub struct App {
    window: Window,
    input: Input,
    sphere_renderer: SphereRenderer,
    mesh_renderer: MeshRenderer,
    camera: Camera,
    controller: Controller,
    timers: FrameTimers,

    simulation: Simulation,

    frame_timer: FrameTimerId,
    cpu_timer: FrameTimerId,
    render_timer: FrameTimerId,
    sim_timer: FrameTimerId,

    world_mesh_id: MeshId,
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
        let sphere_renderer = try!(SphereRenderer::new(&window, Default::default())
            .chain_err(|| "Failed to create SphereRenderer."));
        let mut mesh_renderer = try!(MeshRenderer::new(&window, Default::default())
            .chain_err(|| "Failed to create MeshRenderer."));
        let input = try!(Input::new(&window).chain_err(|| "Failed to create input."));
        let mut camera = Camera::new(75.0, window.aspect_ratio(), 0.1, 100.0);
        let mut timers = FrameTimers::new();
        camera.set_position(Vec3f::new(5.9232, 6.990, 15.044));
        camera.set_yaw(-0.5519);
        camera.set_pitch(0.2919);


        let world_mesh = try!(Heightmap::read("/home/ccc/HMquest03.png", 0.5)
                .chain_err(|| "Failed to load height map."))
            .build_mesh(Vec3f::zero(), Vec3f::new(40.0, 15.0, 40.0));
        let world_mesh_id = try!(mesh_renderer.add(&window, &world_mesh)
            .chain_err(|| "Could not create world mesh."));

        let simulation = Simulation::with_capacity(&mut timers, 16384, &world_mesh);

        Ok(App {
            window: window,
            sphere_renderer: sphere_renderer,
            mesh_renderer: mesh_renderer,
            input: input,
            camera: camera,
            controller: Controller::new(ControllerBindings::default()),

            frame_timer: timers.new_stopped("frame"),
            cpu_timer: timers.new_stopped("cpu"),
            render_timer: timers.new_stopped("render"),
            sim_timer: timers.new_stopped("sim"),
            timers: timers,

            world_mesh_id: world_mesh_id,
            simulation: simulation,
        })
    }

    pub fn run(mut self) -> Result<()> {
        let quit_gesture = Gesture::AnyOf(vec![Gesture::QuitTrigger,
                                               Gesture::KeyTrigger(Scancode::Escape)]);
        let explode_gesture = Gesture::KeyHold(Scancode::E);

        let num_spheres = 100000;
        let mut rng = rand::ChaChaRng::new_unseeded();
        for _ in 0..num_spheres {
            let position = Vec3f::new((rng.gen::<f32>() - 0.5) * 2.0 * 3.0,
                                      rng.gen::<f32>() * 4.0 + 12.0,
                                      (rng.gen::<f32>() - 0.5) * 2.0 * 3.0);
            let velocity = Vec3f::new((rng.gen::<f32>() - 0.5) * 10.,
                                      (rng.gen::<f32>() - 0.5) * 10.,
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
        let meshes = [self.world_mesh_id];
        let colours = [Vec3f::new(0.0, 0.3, 0.4)];
        let transforms = [Mat4::identity()];

        info!("Entering main loop...");
        let mut running = true;
        let start_instant = Instant::now();
        while running {
            for _ in self.simulation.len()..num_spheres {
                let position = Vec3f::new((rng.gen::<f32>() - 0.5) * 2.0 * 15.0,
                                          rng.gen::<f32>() * 5.0 + 30.0,
                                          (rng.gen::<f32>() - 0.5) * 2.0 * 15.0);
                let velocity = Vec3f::new((rng.gen::<f32>() - 0.5) * 10.,
                                          (rng.gen::<f32>() - 0.5) * 10. - 20.0,
                                          (rng.gen::<f32>() - 0.5) * 10.0);
                self.simulation.add(NewEntity {
                    position: position,
                    velocity: velocity,
                    mass: 1.0,
                    radius: 0.1,
                    colour: Vec3f::new(rng.gen::<f32>(), rng.gen::<f32>(), rng.gen::<f32>()) *
                            0.7 + Vec3f::new(0.2, 0.2, 0.2),
                });
            }

            let mut frame = self.window.draw();
            let frame_result = (|| -> Result<()> {
                let delta_time = self.timers.start(self.frame_timer).unwrap_or(1.0 / 60.0);
                self.timers.start(self.cpu_timer);
                self.input.update();

                self.timers.start(self.render_timer);
                try!(self.sphere_renderer
                    .render(&self.window,
                            &self.camera,
                            &mut frame,
                            SphereList {
                                positions: self.simulation.positions(),
                                radii: self.simulation.radii(),
                                colours: self.simulation.colours(),
                            })
                    .chain_err(|| "Failed to render spheres."));
                try!(self.mesh_renderer
                    .render(&self.window,
                            &self.camera,
                            &mut frame,
                            MeshList {
                                ids: &meshes,
                                colours: &colours,
                                transforms: &transforms,
                            })
                    .chain_err(|| "Failed to render meshes."));
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
        let runtime = start_instant.elapsed();
        let runtime = runtime.as_secs() as f64 + runtime.subsec_nanos() as f64 * 1e-9;
        info!("Ran for {:.2}s", runtime);

        Ok(())
    }
}
