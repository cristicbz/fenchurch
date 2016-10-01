use crossbeam;
use gfx::{Window, WindowOptions, Camera, Input, Gesture, Scancode};
use math::{vec3, Vec3f, Mat4};
use num::Zero;
use rand::{self, Rng};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Instant;
use super::controller::{Controller, ControllerBindings};
use super::errors::{Result, ChainErr};
use super::frame_timers::{FrameTimers, FrameTimerId};
use super::heightmap::Heightmap;
use super::mesh_renderer::{MeshRenderer, MeshId, MeshList};
use super::simulation::{Simulation, NewEntity};
use super::snapshot::Snapshot;
use super::sphere_renderer::{SphereRenderer, SphereList};



pub struct App {
    window: Window,
    input: Input,
    sphere_renderer: SphereRenderer,
    mesh_renderer: MeshRenderer,
    camera: Camera,
    controller: Controller,
    timers: FrameTimers,
    sim_timers: FrameTimers,

    simulation: Simulation,

    frame_timer: FrameTimerId,

    world_mesh_id: MeshId,
}

#[derive(Clone)]
struct SimData {
    positions: Vec<Vec3f>,
    radii: Vec<f32>,
    colours: Vec<Vec3f>,
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
        camera.set_position(vec3(5.9232, 6.990, 15.044));
        camera.set_yaw(-0.5519);
        camera.set_pitch(0.2919);


        let world_mesh = try!(Heightmap::read("./HMquest03.png", 0.25)
                .chain_err(|| "Failed to load height map."))
            .build_mesh(Vec3f::zero(), vec3(40.0, 15.0, 40.0));
        let world_mesh_id = try!(mesh_renderer.add(&window, &world_mesh)
            .chain_err(|| "Could not create world mesh."));

        let mut sim_timers = FrameTimers::new();
        let simulation = Simulation::new(&mut sim_timers, &world_mesh, 40000);

        Ok(App {
            window: window,
            sphere_renderer: sphere_renderer,
            mesh_renderer: mesh_renderer,
            input: input,
            camera: camera,
            controller: Controller::new(ControllerBindings::default()),

            frame_timer: timers.new_stopped("frame"),
            timers: timers,
            sim_timers: sim_timers,

            world_mesh_id: world_mesh_id,
            simulation: simulation,
        })
    }

    fn simulation_thread(timers: &mut FrameTimers,
                         simulation: &mut Simulation,
                         sim_data: &Snapshot<SimData>,
                         running: &AtomicBool) {
        let num_spheres = 50_000;
        let mut rng = rand::ChaChaRng::new_unseeded();
        let sim_timer = timers.new_stopped("sim");

        for _ in 0..num_spheres {
            let position = vec3((rng.gen::<f32>() - 0.5) * 2.0 * 10.0,
                                rng.gen::<f32>() * 12.0 + 7.0,
                                (rng.gen::<f32>() - 0.5) * 2.0 * 10.0);
            let velocity = vec3((rng.gen::<f32>() - 0.5) * 2.0 * 15.,
                                rng.gen::<f32>() * 15.,
                                (rng.gen::<f32>() - 0.5) * 2.0 * 15.0);
            simulation.add(NewEntity {
                position: position,
                velocity: velocity,
                mass: 1.0,
                radius: 0.1,
                colour: vec3(1.0, 0.4, 0.05) * (rng.gen::<f32>() + 1.0) * 0.5,
            });
        }
        while running.load(Ordering::Acquire) {
            sim_data.write(|sim_data| {
                sim_data.positions.clear();
                sim_data.radii.clear();
                sim_data.colours.clear();
                sim_data.positions.extend_from_slice(simulation.positions());
                sim_data.radii.extend_from_slice(simulation.radii());
                sim_data.colours.extend_from_slice(simulation.colours());
            });

            let delta_time = timers.start(sim_timer).unwrap_or(0.0);
            for _ in simulation.len()..num_spheres {
                let position = vec3((rng.gen::<f32>() - 0.5) * 2.0 * 1.5 + 1.0,
                                    rng.gen::<f32>() * 0.1 + 10.0,
                                    (rng.gen::<f32>() - 0.5) * 2.0 * 1.5);
                let velocity = vec3((rng.gen::<f32>() - 0.5) * 10.,
                                    rng.gen::<f32>() * 5. + 5.0,
                                    (rng.gen::<f32>() - 0.5) * 10.0);
                simulation.add(NewEntity {
                    position: position,
                    velocity: velocity,
                    mass: 1.0,
                    radius: 0.1,
                    colour: vec3(1.0, 0.4, 0.05) * (rng.gen::<f32>() + 1.0) * 0.5,
                });
            }
            simulation.update(timers, delta_time);
        }
    }

    pub fn run(mut self) -> Result<()> {
        let App { ref mut window,
                  ref mut simulation,
                  ref mut timers,
                  ref mut camera,
                  ref mut input,
                  ref mut sphere_renderer,
                  ref mut mesh_renderer,
                  ref mut controller,
                  world_mesh_id,
                  frame_timer,
                  ref mut sim_timers,
                  .. } = self;
        let quit_gesture = Gesture::AnyOf(vec![Gesture::QuitTrigger,
                                               Gesture::KeyTrigger(Scancode::Escape)]);


        let meshes = [world_mesh_id];
        let colours = [vec3(0.0, 0.3, 0.4)];
        let transforms = [Mat4::identity()];
        let sim_data = Snapshot::new(SimData {
            positions: Vec::new(),
            radii: Vec::new(),
            colours: Vec::new(),
        });

        info!("Entering main loop...");
        let mut running = true;
        let simulation_running_flag = AtomicBool::new(true);
        let start_instant = Instant::now();
        let result = crossbeam::scope(|scope| -> Result<()> {
            scope.spawn(|| {
                App::simulation_thread(sim_timers, simulation, &sim_data, &simulation_running_flag);
            });
            while running {
                timers.query(frame_timer).map(|time| {
                    let sleep_for = 1.0 / 25.0 - time;
                    if sleep_for > 0.0 {
                        ::std::thread::sleep(::std::time::Duration::from_millis(
                                (sleep_for * 1e3) as u64))
                    }
                });
                let delta_time = timers.start(frame_timer).unwrap_or(1.0 / 60.0);
                let mut frame = window.draw();
                let frame_result = (|| -> Result<()> {
                    input.update();

                    try!(sim_data.read(|sim_data| {
                            sphere_renderer.update(window,
                                        SphereList {
                                            positions: &sim_data.positions,
                                            radii: &sim_data.radii,
                                            colours: &sim_data.colours,
                                        })
                                .chain_err(|| "Failed to update spheres.")
                        })
                        .unwrap_or(Ok(())));

                    try!(sphere_renderer.render(camera, &mut frame)
                        .chain_err(|| "Failed to render spheres."));

                    try!(mesh_renderer.render(window,
                                camera,
                                &mut frame,
                                MeshList {
                                    ids: &meshes,
                                    colours: &colours,
                                    transforms: &transforms,
                                })
                        .chain_err(|| "Failed to render meshes."));

                    if input.poll_gesture(&quit_gesture) {
                        running = false;
                        simulation_running_flag.store(false, Ordering::Release);
                    }
                    controller.update(delta_time, input, camera);
                    Ok(())
                })();
                try!(frame.finish().chain_err(|| "Context lost."));
                try!(frame_result);
            }
            Ok(())
        });
        let runtime = start_instant.elapsed();
        let runtime = runtime.as_secs() as f64 + runtime.subsec_nanos() as f64 * 1e-9;
        info!("Ran for {:.2}s", runtime);

        result
    }
}
