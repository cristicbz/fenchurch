use gfx::{Window, Camera, GLSL_VERSION_STRING};
use glium::index::{PrimitiveType, NoIndices};
use glium::program::ProgramCreationInput;
use glium::{Program, Frame, Surface, DrawParameters, BackfaceCullingMode, Depth, DepthTest};
use glium::texture::{Texture1d, UncompressedFloatFormat, MipmapsOption};
use glium::uniforms::{SamplerWrapFunction, MagnifySamplerFilter, MinifySamplerFilter};
use glium::vertex::EmptyVertexAttributes;
use math::{Vec3f, Vector};
use super::errors::{Result, ChainErr};
use super::utils::read_utf8_file;
use std::u32;

pub struct SphereList<'a> {
    pub positions: &'a [Vec3f],
    pub radii: &'a [f32],
    pub colours: &'a [Vec3f],
}

pub struct SphereRenderer {
    program: Program,
    draw_parameters: DrawParameters<'static>,
    settings: Settings,
}

pub struct Light {
    direction: Vec3f,
    colour: Vec3f,
}

pub struct Settings {
    pub key_light: Light,
    pub fill_light: Light,
    pub back_light: Light,
    pub ambient: Vec3f,
    pub chunk_size: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            key_light: Light {
                direction: Vec3f::new(0.0, 1.0, 1.0),
                colour: Vec3f::new(1.0, 1.0, 1.0) * 1.0,
            },
            fill_light: Light {
                direction: Vec3f::new(1.0, 1.0, 0.0),
                colour: Vec3f::new(1.0, 0.8, 0.7) * 0.4,
            },
            back_light: Light {
                direction: Vec3f::new(-1.0, -1.0, 0.0),
                colour: Vec3f::new(0.7, 0.8, 1.0) * 0.1,
            },
            ambient: Vec3f::new(0.2, 1.0, 1.0) * 0.05,
            chunk_size: 1024,
        }
    }
}

impl SphereRenderer {
    pub fn new(window: &Window, settings: Settings) -> Result<Self> {
        assert!(settings.chunk_size > 1);

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

        Ok(SphereRenderer {
            program: program,
            settings: settings,
            draw_parameters: DrawParameters {
                depth: Depth {
                    test: DepthTest::IfLess,
                    write: true,
                    ..Depth::default()
                },
                backface_culling: BackfaceCullingMode::CullClockwise,
                ..DrawParameters::default()
            },
        })
    }

    pub fn render<'a>(&self,
                      window: &Window,
                      camera: &Camera,
                      frame: &mut Frame,
                      sphere_list: SphereList<'a>)
                      -> Result<()> {

        let SphereList { positions, radii, colours } = sphere_list;
        let num_spheres = positions.len();
        assert_eq!(radii.len(), num_spheres);
        assert_eq!(colours.len(), num_spheres);

        assert!(positions.len() <= u32::MAX as usize);

        let chunk_size = self.settings.chunk_size;
        let chunks = positions.chunks(chunk_size)
            .zip(radii.chunks(chunk_size))
            .zip(colours.chunks(chunk_size))
            .map(|((position_chunk, radius_chunk), colour_chunk)| {
                SphereList {
                    positions: position_chunk,
                    radii: radius_chunk,
                    colours: colour_chunk,
                }
            });

        let modelview = camera.modelview();
        let projection = camera.projection();

        let key_light_direction =
            modelview.transform_direction(&self.settings.key_light.direction).xyz().normalized();
        let fill_light_direction =
            modelview.transform_direction(&self.settings.fill_light.direction).xyz().normalized();
        let back_light_direction =
            modelview.transform_direction(&self.settings.back_light.direction).xyz().normalized();

        for chunk in chunks {
            let positions = try!(Texture1d::with_format(window.facade(),
                                                        chunk.positions,
                                                        UncompressedFloatFormat::F32F32F32,
                                                        MipmapsOption::NoMipmap)
                .chain_err(|| "Failed to build positions texture."));
            let radii = try!(Texture1d::with_format(window.facade(),
                                                    chunk.radii,
                                                    UncompressedFloatFormat::F32,
                                                    MipmapsOption::NoMipmap)
                .chain_err(|| "Failed to build radii texture."));
            let colours = try!(Texture1d::with_format(window.facade(),
                                                      chunk.colours,
                                                      UncompressedFloatFormat::F32F32F32,
                                                      MipmapsOption::NoMipmap)
                .chain_err(|| "Failed to build colours texture."));

            let uniforms = uniform! {
                u_modelview: &modelview,
                u_projection: projection,
                u_key_light_direction: &key_light_direction,
                u_key_light_colour: &self.settings.key_light.colour,
                u_fill_light_direction: &fill_light_direction,
                u_fill_light_colour: &self.settings.fill_light.colour,
                u_back_light_direction: &back_light_direction,
                u_back_light_colour: &self.settings.back_light.colour,
                u_ambient_colour: &self.settings.ambient,
                u_positions: positions.sampled()
                                      .magnify_filter(MagnifySamplerFilter::Nearest)
                                      .minify_filter(MinifySamplerFilter::Nearest)
                                      .wrap_function(SamplerWrapFunction::Clamp),
                u_radii: radii.sampled()
                              .magnify_filter(MagnifySamplerFilter::Nearest)
                              .minify_filter(MinifySamplerFilter::Nearest)
                              .wrap_function(SamplerWrapFunction::Clamp),
                u_colours: colours.sampled()
                                  .magnify_filter(MagnifySamplerFilter::Nearest)
                                  .minify_filter(MinifySamplerFilter::Nearest)
                                  .wrap_function(SamplerWrapFunction::Clamp),
            };

            try!(frame.draw(EmptyVertexAttributes { len: chunk.positions.len() * 6 },
                      NoIndices(PrimitiveType::TrianglesList),
                      &self.program,
                      &uniforms,
                      &self.draw_parameters)
                .chain_err(|| "Failed to render frame."));
        }

        Ok(())
    }
}


const VERTEX_SHADER: &'static str = "src/shaders/sphere.vert";
const FRAGMENT_SHADER: &'static str = "src/shaders/sphere.frag";
