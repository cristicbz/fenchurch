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

pub struct Settings {
    pub light_direction: Vec3f,
    pub chunk_size: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            light_direction: Vec3f::new(0.577f32, 0.577f32, 0.577f32),
            chunk_size: 1024,
        }
    }
}

impl SphereRenderer {
    pub fn new(window: &Window, mut settings: Settings) -> Result<Self> {
        assert!(settings.chunk_size > 1);
        settings.light_direction.normalize();

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

        let Settings { light_direction, chunk_size } = self.settings;
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
                u_light_direction: &light_direction,
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
