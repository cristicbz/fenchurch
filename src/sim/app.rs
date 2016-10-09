use gfx::{Window, WindowOptions, Camera, Input, Gesture, Scancode};
use math::{vec3, Vec3f, Mat4};
use num::Zero;
use rand::{self, Rng};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Instant;
use super::controller::{Controller, ControllerBindings};
use super::errors::{Result, ChainErr};
use super::frame_timers::{FrameTimers, FrameTimerId};
use super::heightmap::Heightmap;
use super::mesh_renderer::{MeshRenderer, MeshId, MeshList, Mesh};
use super::pods::{PodSystem, Pod};
use super::simulation::{Simulation, Body};
use super::snapshot::Snapshot;
use super::sphere_renderer::{SphereRenderer, SphereList};
use std::mem;

#[derive(Clone)]
struct SimData {
    positions: Vec<Vec3f>,
    radii: Vec<f32>,
    colours: Vec<Vec3f>,
}

struct SimulatorThread {
    pods: PodSystem,
    simulation: Simulation,
    timers: FrameTimers,
    shared: Arc<Shared>,
}

struct Shared {
    running: AtomicBool,
    data: Snapshot<SimData>,
}

impl SimulatorThread {
    fn new(world_mesh: &Mesh) -> Self {
        let mut timers = FrameTimers::new();
        let shared = Arc::new(Shared {
            running: AtomicBool::new(true),
            data: Snapshot::new(SimData {
                positions: Vec::new(),
                radii: Vec::new(),
                colours: Vec::new(),
            }),
        });

        SimulatorThread {
            simulation: Simulation::new(&mut timers, world_mesh),
            pods: PodSystem::with_capacity(40_000),
            timers: timers,
            shared: shared,
        }
    }

    fn run(&mut self) {
        let SimulatorThread { ref mut pods, ref mut simulation, ref mut timers, ref shared } =
            *self;
        let Shared { ref running, ref data } = **shared;

        let num_spheres = 40_000;
        let mut rng = rand::ChaChaRng::new_unseeded();
        let sim_timer = timers.new_stopped("sim");

        for _ in 0..num_spheres {
            let position = vec3((rng.gen::<f32>() - 0.5) * 2.0 * 10.0,
                                rng.gen::<f32>() * 12.0 + 7.0,
                                (rng.gen::<f32>() - 0.5) * 2.0 * 10.0);
            let velocity = vec3((rng.gen::<f32>() - 0.5) * 2.0 * 15.,
                                rng.gen::<f32>() * 15.,
                                (rng.gen::<f32>() - 0.5) * 2.0 * 15.0);
            pods.add(Pod {
                colour: vec3(1.0, 0.4, 0.05) * (rng.gen::<f32>() + 1.0) * 0.5,
                body: Body::new(position, velocity, 0.1, 1.0),
            });
        }
        while running.load(Ordering::Acquire) {
            data.write(|data| {
                let pods = pods.pods();
                data.positions.clear();
                data.radii.clear();
                data.colours.clear();
                data.positions.extend_from_slice(pods.bodies.positions);
                data.radii.extend_from_slice(pods.bodies.radii);
                data.colours.extend_from_slice(pods.colours);
            });

            let delta_time = timers.start(sim_timer).unwrap_or(0.0);
            for _ in pods.len()..num_spheres {
                let position = vec3((rng.gen::<f32>() - 0.5) * 2.0 * 1.5 + 1.0,
                                    rng.gen::<f32>() * 0.1 + 10.0,
                                    (rng.gen::<f32>() - 0.5) * 2.0 * 1.5);
                let velocity = vec3((rng.gen::<f32>() - 0.5) * 10.,
                                    rng.gen::<f32>() * 5. + 5.0,
                                    (rng.gen::<f32>() - 0.5) * 10.0);
                pods.add(Pod {
                    colour: vec3(1.0, 0.4, 0.05) * (rng.gen::<f32>() + 1.0) * 0.5,
                    body: Body::new(position, velocity, 0.1, 1.0),
                });
            }
            simulation.update(timers, &mut pods.pods_mut().bodies);
        }


    }
}

enum State {
    Stopped(SimulatorThread),
    Running(JoinHandle<SimulatorThread>),
    Busy,
}

pub struct Simulator {
    state: State,
    shared: Arc<Shared>,
}

impl Simulator {
    pub fn new(world_mesh: &Mesh) -> Self {
        let thread = SimulatorThread::new(world_mesh);
        let shared = thread.shared.clone();
        Simulator {
            state: State::Stopped(thread),
            shared: shared,
        }
    }

    pub fn data(&self) -> &Snapshot<SimData> {
        &self.shared.data
    }

    pub fn run(&mut self) {
        match mem::replace(&mut self.state, State::Busy) {
            State::Stopped(mut thread) => {
                self.state = State::Running(thread::spawn(|| {
                    thread.run();
                    thread
                }));
            }
            other @ _ => self.state = other,
        }
    }

    pub fn stop(&mut self) {
        self.shared.running.store(false, Ordering::Release);

        match mem::replace(&mut self.state, State::Busy) {
            State::Running(thread) => {
                self.state = State::Stopped(thread.join().expect("panic in simulator thread"));
            }
            other @ _ => self.state = other,
        }
    }
}


pub struct App {
    window: Window,
    input: Input,
    sphere_renderer: SphereRenderer,
    mesh_renderer: MeshRenderer,
    camera: Camera,
    controller: Controller,
    timers: FrameTimers,
    frame_timer: FrameTimerId,
    world_mesh_id: MeshId,

    simulator: Simulator,
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

        Ok(App {
            window: window,
            sphere_renderer: sphere_renderer,
            mesh_renderer: mesh_renderer,
            input: input,
            camera: camera,
            controller: Controller::new(ControllerBindings::default()),

            frame_timer: timers.new_stopped("frame"),
            timers: timers,

            world_mesh_id: world_mesh_id,
            simulator: Simulator::new(&world_mesh),
        })
    }


    pub fn run(mut self) -> Result<()> {
        let App { ref mut window,
                  ref mut simulator,
                  ref mut timers,
                  ref mut camera,
                  ref mut input,
                  ref mut sphere_renderer,
                  ref mut mesh_renderer,
                  ref mut controller,
                  world_mesh_id,
                  frame_timer,
                  .. } = self;
        let quit_gesture = Gesture::AnyOf(vec![Gesture::QuitTrigger,
                                               Gesture::KeyTrigger(Scancode::Escape)]);


        let meshes = [world_mesh_id];
        let colours = [vec3(0.0, 0.3, 0.4)];
        let transforms = [Mat4::identity()];

        info!("Entering main loop...");
        let mut running = true;
        let start_instant = Instant::now();
        simulator.run();
        while running {
            let delta_time = timers.start(frame_timer).unwrap_or(1.0 / 60.0);
            let mut frame = window.draw();
            let frame_result = (|| -> Result<()> {
                input.update();

                try!(simulator.data()
                    .read(|data| {
                        sphere_renderer.update(window,
                                    SphereList {
                                        positions: &data.positions,
                                        radii: &data.radii,
                                        colours: &data.colours,
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
                    simulator.stop();
                }
                controller.update(delta_time, input, camera);
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
