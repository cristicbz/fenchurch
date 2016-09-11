use gfx::{Window, WindowOptions, Camera, Input, Analog2d, Gesture, Scancode, GLSL_VERSION_STRING,
          MouseButton};
use glium::index::{PrimitiveType, IndexBuffer};
use glium::{Program, VertexBuffer, Frame, Surface, DrawParameters, BackfaceCullingMode, Depth,
            DepthTest};
use glium::program::ProgramCreationInput;
use super::errors::{Result, ChainErr};
use math::{Vector, Vec3f};
use std::fmt::Write;

use std::fs::File;
use std::io::Read;
use std::path::Path;
use std::f32::consts::FRAC_PI_2;
use std::time::{Instant, Duration};
use slab::Slab;
use std::borrow::Cow;
use std::mem;

pub struct App {
    window: Window,
    input: Input,
    renderer: Renderer,
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
        let renderer = try!(Renderer::new(&window).chain_err(|| "Failed to create renderer."));
        let input = try!(Input::new(&window).chain_err(|| "Failed to create input."));
        let mut camera = Camera::new(75.0, window.aspect_ratio(), 0.01, 100.0);
        let mut timers = FrameTimers::new();
        camera.set_position(Vec3f::new(0.0, 0.0, 5.0));

        let frame_timer = timers.new_stopped("frame");
        let cpu_timer = timers.new_stopped("cpu");

        Ok(App {
            window: window,
            renderer: renderer,
            input: input,
            camera: camera,
            controller: Controller::new(),
            timers: timers,
            frame_timer: frame_timer,
            cpu_timer: cpu_timer,
        })
    }

    pub fn run(mut self) -> Result<()> {
        let quit_gesture = Gesture::AnyOf(vec![Gesture::QuitTrigger,
                                               Gesture::KeyTrigger(Scancode::Escape)]);

        info!("Entering main loop...");
        let mut running = true;
        while running {
            let delta_time = self.timers.start(self.frame_timer).unwrap_or(1.0 / 60.0);
            let mut frame = self.window.draw();
            self.timers.start(self.cpu_timer);
            self.input.update();
            try!(self.renderer
                .render(&self.camera, &mut frame)
                .chain_err(|| "Failed to render frame."));
            if self.input.poll_gesture(&quit_gesture) {
                running = false;
            }
            self.controller.update(delta_time, &mut self.input, &mut self.camera);
            self.timers.stop(self.cpu_timer);
            try!(frame.finish().chain_err(|| "Context lost."));
        }

        Ok(())
    }
}

pub struct ControllerBindings {
    pub movement: Analog2d,
    pub look: Analog2d,
    pub jump: Gesture,
    pub toggle_movement: Gesture,
}

impl Default for ControllerBindings {
    fn default() -> Self {
        ControllerBindings {
            movement: Analog2d::Gestures {
                x_positive: Gesture::KeyHold(Scancode::D),
                x_negative: Gesture::KeyHold(Scancode::A),
                y_positive: Gesture::KeyHold(Scancode::W),
                y_negative: Gesture::KeyHold(Scancode::S),
                step: 1.0,
            },
            look: Analog2d::Sum {
                analogs: vec![
                    Analog2d::Gestures {
                        x_positive: Gesture::KeyHold(Scancode::Right),
                        x_negative: Gesture::KeyHold(Scancode::Left),
                        y_positive: Gesture::KeyHold(Scancode::Down),
                        y_negative: Gesture::KeyHold(Scancode::Up),
                        step: 0.05,
                    },
                    Analog2d::Mouse {
                        sensitivity: 0.004
                    },
                ],
            },
            jump: Gesture::KeyHold(Scancode::Space),
            toggle_movement: Gesture::ButtonHold(MouseButton::Left),
        }
    }
}

pub struct Controller {
    bindings: ControllerBindings,
    move_speed: f32,
    moving: bool,
}

impl Controller {
    fn new() -> Self {
        Controller {
            bindings: ControllerBindings::default(),
            move_speed: 0.1,
            moving: false,
        }
    }

    fn update(&mut self, _delta_time: f32, input: &mut Input, camera: &mut Camera) {
        if input.poll_gesture(&self.bindings.toggle_movement) {
            if !self.moving {
                self.moving = true;
                input.set_cursor_grabbed(true);
            }
        } else {
            if self.moving {
                self.moving = false;
                input.set_cursor_grabbed(false);
            }
            return;
        }

        let movement = input.poll_analog2d(&self.bindings.movement);
        let look = input.poll_analog2d(&self.bindings.look);
        let jump = input.poll_gesture(&self.bindings.jump);
        let yaw = camera.yaw() + look[0];
        let pitch = clamp(camera.pitch() + look[1], (-FRAC_PI_2, FRAC_PI_2));
        camera.set_yaw(yaw);
        camera.set_pitch(pitch);

        let up = if jump { 0.5 } else { 0.0 };
        let movement =
            Vec3f::new(yaw.cos() * movement[0] + yaw.sin() * movement[1] * pitch.cos(),
                       -pitch.sin() * movement[1] + up,
                       -yaw.cos() * movement[1] * pitch.cos() + yaw.sin() * movement[0])
                .normalized() * self.move_speed;
        camera.move_by(movement);
    }
}

#[derive(Copy, Clone, Debug, Hash, Eq, PartialEq, Ord, PartialOrd)]
pub struct FrameTimerId(usize);

struct FrameTimer {
    debug_name: Cow<'static, str>,
    last_start: Option<Instant>,

    seconds_since_logged: f32,
    times_since_logged: f32,
}

pub struct FrameTimers {
    timers: Slab<FrameTimer>,
    last_logged: Option<Instant>,
}

impl FrameTimers {
    pub fn new() -> Self {
        FrameTimers {
            timers: Slab::with_capacity(16),
            last_logged: None,
        }
    }

    pub fn new_stopped<S: Into<Cow<'static, str>>>(&mut self, debug_name: S) -> FrameTimerId {
        match self.timers.insert(FrameTimer {
            debug_name: debug_name.into(),
            last_start: None,
            seconds_since_logged: 0.0,
            times_since_logged: 0.0,
        }) {
            Ok(index) => FrameTimerId(index),
            Err(timer) => panic!("Too many timers: {}", timer.debug_name),

        }
    }

    pub fn remove(&mut self, timer_id: FrameTimerId) {
        self.timers.remove(timer_id.0).expect("Invalid timer id.");
    }

    pub fn start(&mut self, timer_id: FrameTimerId) -> Option<f32> {
        let time = {
            let &mut FrameTimer { ref mut last_start,
                                  ref mut seconds_since_logged,
                                  ref mut times_since_logged,
                                  .. } = &mut self.timers[timer_id.0];
            let current_time = Instant::now();
            mem::replace(last_start, Some(current_time)).map(|last_start| {
                let elapsed = duration_to_seconds(current_time.duration_since(last_start));
                *seconds_since_logged += elapsed;
                *times_since_logged += 1.0;
                elapsed
            })
        };
        self.maybe_log();
        time
    }

    pub fn stop(&mut self, timer_id: FrameTimerId) -> Option<f32> {
        let time = {
            let &mut FrameTimer { ref mut last_start,
                                  ref mut seconds_since_logged,
                                  ref mut times_since_logged,
                                  .. } = &mut self.timers[timer_id.0];
            mem::replace(last_start, None).map(|last_start| {
                let elapsed = duration_to_seconds(last_start.elapsed());
                *seconds_since_logged += elapsed;
                *times_since_logged += 1.0;
                elapsed
            })
        };
        self.maybe_log();
        time
    }

    pub fn maybe_log(&mut self) {
        let current_time = Instant::now();
        match self.last_logged.map(|last_logged| current_time.duration_since(last_logged)) {
            Some(duration) if duration.as_secs() >= 5 => {
                self.last_logged = Some(current_time);
            }
            None => {
                self.last_logged = Some(current_time);
                return;
            }
            Some(_) => return,
        };

        info!("Frame timer summary:{}",
              self.timers
                  .iter_mut()
                  .fold(String::with_capacity(256), |mut output,
                         &mut FrameTimer { ref debug_name,
                                           ref mut seconds_since_logged,
                                           ref mut times_since_logged,
                                           .. }| {
                let seconds_since_logged = mem::replace(seconds_since_logged, 0.0);
                let times_since_logged = mem::replace(times_since_logged, 0.0);
                let _ = write!(&mut output,
                               "\n\t{}\t{:.2}/s (avg {:.2}ms)",
                               debug_name,
                               times_since_logged / seconds_since_logged,
                               seconds_since_logged / times_since_logged * 1000.);
                output
            }));
    }
}

fn duration_to_seconds(duration: Duration) -> f32 {
    duration.as_secs() as f32 + (duration.subsec_nanos() as f32 * 1e-9f32)
}


pub struct Renderer {
    program: Program,
    buffer: VertexBuffer<SphereVertex>,
    indices: IndexBuffer<u32>,
    draw_parameters: DrawParameters<'static>,
}

impl Renderer {
    fn new(window: &Window) -> Result<Self> {
        let program = try!(Program::new(window.facade(),
                                        ProgramCreationInput::SourceCode {
                                            vertex_shader: &format!("#version {}\n{}",
                                                          GLSL_VERSION_STRING,
                                                          try!(read_utf8_file(VERTEX_SHADER)
                                                              .chain_err(|| {
                                                                  "Failed to read vertex shader."
                                                              }))),
                                            tessellation_control_shader: None,
                                            tessellation_evaluation_shader: None,
                                            geometry_shader: None,
                                            fragment_shader: &format!("#version {}\n{}",
                                                            GLSL_VERSION_STRING,
                                                            try!(read_utf8_file(FRAGMENT_SHADER)
                                                                .chain_err(|| {
                                                                    "Failed to read fragment \
                                                                     shader."
                                                                }))),
                                            transform_feedback_varyings: None,
                                            outputs_srgb: false,
                                            uses_point_size: false,
                                        })
            .chain_err(|| "Failed to build program."));

        let spheres = vec![
            Sphere { position: [0.0, 0.0, 0.0], radius: 1.0 },
            Sphere { position: [0.0, 1.0, 0.0], radius: 0.5 },
            Sphere { position: [0.0, 1.5, 0.0], radius: 0.25 },

            Sphere { position: [0.0, 0.0, 0.0], radius: 1.0 },
            Sphere { position: [1.0, 0.0, 0.0], radius: 0.5 },
            Sphere { position: [1.5, 0.0, 0.0], radius: 0.25 },

            Sphere { position: [0.0, 0.0, 0.0], radius: 1.0 },
            Sphere { position: [0.0, 0.0, 1.0], radius: 0.5 },
            Sphere { position: [0.0, 0.0, 1.5], radius: 0.25 },

            Sphere { position: [0.0, 0.0, 0.0], radius: 1.0 },
            Sphere { position: [0.0, -1.0, 0.0], radius: 0.5 },
            Sphere { position: [0.0, -1.5, 0.0], radius: 0.25 },

            Sphere { position: [0.0, 0.0, 0.0], radius: 1.0 },
            Sphere { position: [-1.0, 0.0, 0.0], radius: 0.5 },
            Sphere { position: [-1.5, 0.0, 0.0], radius: 0.25 },

            Sphere { position: [0.0, 0.0, 0.0], radius: 1.0 },
            Sphere { position: [0.0, 0.0, -1.0], radius: 0.5 },
            Sphere { position: [0.0, 0.0, -1.5], radius: 0.25 },
        ];

        let mut vertices = Vec::with_capacity(spheres.len() * 4);
        let mut indices = Vec::with_capacity(spheres.len() * 6);

        for sphere in &spheres {
            let start_vertex_index = vertices.len() as u32;
            for corner_index in 0..4 {
                vertices.push(SphereVertex {
                    a_position: sphere.position,
                    a_radius: sphere.radius,
                    a_corner_index: corner_index,
                });
            }

            indices.extend_from_slice(&[start_vertex_index,
                                        start_vertex_index + 1,
                                        start_vertex_index + 2]);
            indices.extend_from_slice(&[start_vertex_index + 1,
                                        start_vertex_index + 3,
                                        start_vertex_index + 2]);
        }

        Ok(Renderer {
            program: program,
            buffer: try!(VertexBuffer::immutable(window.facade(), &vertices)
                .chain_err(|| "Failed to build vertex buffer")),
            indices: try!(IndexBuffer::immutable(window.facade(),
                                                 PrimitiveType::TrianglesList,
                                                 &indices)
                .chain_err(|| "Failed to build index buffer")),
            draw_parameters: DrawParameters {
                depth: Depth {
                    test: DepthTest::IfLess,
                    write: true,
                    ..Depth::default()
                },
                backface_culling: BackfaceCullingMode::CullCounterClockwise,
                ..DrawParameters::default()
            },
        })
    }

    fn render(&self, camera: &Camera, frame: &mut Frame) -> Result<()> {
        let modelview = camera.modelview();
        let uniforms = uniform! {
            u_modelview: &modelview,
            u_projection: camera.projection(),
            u_light_direction: [0.577f32, 0.577f32, 0.577],
        };

        try!(frame.draw(&self.buffer,
                  &self.indices,
                  &self.program,
                  &uniforms,
                  &self.draw_parameters)
            .chain_err(|| "Failed to render frame."));

        Ok(())
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Sphere {
    pub position: [f32; 3],
    pub radius: f32,
}

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct SphereVertex {
    pub a_position: [f32; 3],
    pub a_radius: f32,
    pub a_corner_index: u8,
}

const VERTEX_SHADER: &'static str = "src/shaders/sphere.vert";
const FRAGMENT_SHADER: &'static str = "src/shaders/sphere.frag";

implement_vertex!(SphereVertex, a_position, a_radius, a_corner_index);

fn clamp<T: PartialOrd>(value: T, (limit_min, limit_max): (T, T)) -> T {
    if value < limit_min {
        limit_min
    } else if value > limit_max {
        limit_max
    } else {
        value
    }
}

fn read_utf8_file<P: AsRef<Path>>(path: P) -> Result<String> {
    let path = path.as_ref();
    let mut output = String::new();
    let mut file = try!(File::open(path).chain_err(|| format!("Error opening {:?}", path)));
    try!(file.read_to_string(&mut output).chain_err(|| format!("Error reading {:?}", path)));
    Ok(output)
}
