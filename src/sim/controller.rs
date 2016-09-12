use gfx::{Camera, Input, Analog2d, Gesture, Scancode, MouseButton};
use math::{Vector, Vec3f, clamp};
use std::f32::consts::FRAC_PI_2;

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
    pub fn new(bindings: ControllerBindings) -> Self {
        Controller {
            bindings: bindings,
            move_speed: 6.0,
            moving: false,
        }
    }

    pub fn update(&mut self, delta_time: f32, input: &mut Input, camera: &mut Camera) {
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
                .normalized() * self.move_speed * delta_time;
        camera.move_by(movement);
    }
}
