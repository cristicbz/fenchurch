use glium::{Frame, Surface};
use glium_sdl2::{DisplayBuild, SDL2Facade};
use sdl2;
use sdl2::Sdl;
use sdl2::video::GLProfile;
use std::borrow::Cow;
use super::errors::*;

#[cfg(target_os = "linux")]
mod internal {
    pub const GL_MAJOR_VERSION: u8 = 3;
    pub const GL_MINOR_VERSION: u8 = 3;
    pub const GLSL_VERSION_STRING: &'static str = "330 core";
}

#[cfg(not(target_os = "linux"))]
mod internal {
    pub const GL_MAJOR_VERSION: u8 = 3;
    pub const GL_MINOR_VERSION: u8 = 3;
    pub const GLSL_VERSION_STRING: &'static str = "330 core";
}

pub use self::internal::{GL_MAJOR_VERSION, GL_MINOR_VERSION, GLSL_VERSION_STRING};

pub struct Options<'a> {
    pub title: Cow<'a, str>,
    pub width: u32,
    pub height: u32,
    pub depth: u8,
    pub background: (f32, f32, f32, f32),
}

pub struct Window {
    sdl: Sdl,
    facade: SDL2Facade,
    width: u32,
    height: u32,
    background: (f32, f32, f32, f32),
}

impl<'a> Default for Options<'a> {
    fn default() -> Options<'a> {
        Options {
            title: "<unnamed window>".into(),
            width: 100,
            height: 100,
            depth: 24,
            background: (0.0, 0.0, 0.0, 0.0),
        }
    }
}

impl Window {
    pub fn new<'a>(options: Options) -> Result<Window> {
        let Options { title, width, height, depth, background } = options;

        let sdl = try!(sdl2::init().map_err(SdlError).chain_err(|| "Sdl init failed."));
        let video = try!(sdl.video()
            .map_err(SdlError)
            .chain_err(|| "Sdl video init failed."));
        let gl_attr = video.gl_attr();
        gl_attr.set_context_profile(GLProfile::Core);
        gl_attr.set_context_major_version(GL_MAJOR_VERSION);
        gl_attr.set_context_minor_version(GL_MINOR_VERSION);
        gl_attr.set_depth_size(depth);
        gl_attr.set_double_buffer(true);

        let facade = try!(video.window(&title, width, height)
            .position_centered()
            .opengl()
            .resizable()
            .build_glium()
            .chain_err(|| "SDL Window creation failed."));

        sdl2::clear_error();
        Ok(Window {
            sdl: sdl,
            facade: facade,
            width: width,
            height: height,
            background: background,
        })
    }

    pub fn sdl(&self) -> &Sdl {
        &self.sdl
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn aspect_ratio(&self) -> f32 {
        self.width as f32 / self.height as f32
    }

    pub fn draw(&self) -> Frame {
        let mut frame = self.facade.draw();
        frame.clear_all_srgb(self.background, 1.0, 0);
        frame
    }

    pub fn facade(&self) -> &SDL2Facade {
        &self.facade
    }
}
