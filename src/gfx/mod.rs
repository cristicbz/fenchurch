pub mod camera;
pub mod input;
pub mod window;
pub mod errors;

pub use self::window::{Window, Options as WindowOptions, GLSL_VERSION_STRING};
pub use self::camera::Camera;
pub use self::input::{Input, Gesture, Analog2d, Scancode, MouseButton};
pub use self::errors::{SdlError, Error};

use glium::uniforms::{AsUniformValue, UniformValue};
use glium::texture::{ClientFormat, PixelValue};
use glium::vertex::{Attribute, AttributeType};
use math::{Vec2f, Vec3f, Vec4f, Mat4};


impl AsUniformValue for Mat4 {
    fn as_uniform_value(&self) -> UniformValue {
        UniformValue::Mat4([[self[0][0], self[0][1], self[0][2], self[0][3]],
                            [self[1][0], self[1][1], self[1][2], self[1][3]],
                            [self[2][0], self[2][1], self[2][2], self[2][3]],
                            [self[3][0], self[3][1], self[3][2], self[3][3]]])
    }
}

impl<'a> AsUniformValue for &'a Mat4 {
    fn as_uniform_value(&self) -> UniformValue {
        UniformValue::Mat4([[self[0][0], self[0][1], self[0][2], self[0][3]],
                            [self[1][0], self[1][1], self[1][2], self[1][3]],
                            [self[2][0], self[2][1], self[2][2], self[2][3]],
                            [self[3][0], self[3][1], self[3][2], self[3][3]]])
    }
}

impl<'a> AsUniformValue for &'a Vec3f {
    fn as_uniform_value(&self) -> UniformValue {
        UniformValue::Vec3([self[0], self[1], self[2]])
    }
}

unsafe impl PixelValue for Vec2f {
    fn get_format() -> ClientFormat {
        ClientFormat::F32F32
    }
}

unsafe impl PixelValue for Vec3f {
    fn get_format() -> ClientFormat {
        ClientFormat::F32F32F32
    }
}

unsafe impl PixelValue for Vec4f {
    fn get_format() -> ClientFormat {
        ClientFormat::F32F32F32F32
    }
}

unsafe impl Attribute for Vec2f {
    fn get_type() -> AttributeType {
        AttributeType::F32F32
    }
}

unsafe impl Attribute for Vec3f {
    fn get_type() -> AttributeType {
        AttributeType::F32F32F32
    }
}

unsafe impl Attribute for Vec4f {
    fn get_type() -> AttributeType {
        AttributeType::F32F32F32F32
    }
}
