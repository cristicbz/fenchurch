use gfx::{Window, Camera};
use glium::index::{PrimitiveType, NoIndices};
use glium::{Program, Frame, Surface, DrawParameters, BackfaceCullingMode, Depth, DepthTest};
use glium::texture::{Texture1d, UncompressedFloatFormat, MipmapsOption};
use glium::uniforms::{SamplerWrapFunction, MagnifySamplerFilter, MinifySamplerFilter};
use glium::vertex::EmptyVertexAttributes;
use math::Vec3f;
use super::errors::{Result, ChainErr};
use super::lights::Lights;
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
    chunks: Vec<Chunk>,
}

struct Chunk {
    positions: Texture1d,
    radii: Texture1d,
    colours: Texture1d,
    size: usize,
}

pub struct Settings {
    pub lights: Lights,
    pub chunk_size: usize,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            lights: Lights::default(),
            chunk_size: 1024,
        }
    }
}

impl SphereRenderer {
    pub fn new(window: &Window, settings: Settings) -> Result<Self> {
        assert!(settings.chunk_size > 1);

        let program = try!(window.program(VERTEX_SHADER, FRAGMENT_SHADER)
            .chain_err(|| "Failed to build SphereRenderer program."));

        Ok(SphereRenderer {
            program: program,
            settings: settings,
            chunks: Vec::new(),
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

    pub fn render<'a>(&mut self,
                      window: &Window,
                      camera: &Camera,
                      frame: &mut Frame,
                      sphere_list: Option<SphereList<'a>>)
                      -> Result<()> {

        if let Some(SphereList { positions, radii, colours }) = sphere_list {
            let num_spheres = positions.len();
            assert_eq!(radii.len(), num_spheres);
            assert_eq!(colours.len(), num_spheres);
            assert!(positions.len() <= u32::MAX as usize);

            let chunk_size = self.settings.chunk_size;
            self.chunks.clear();
            self.chunks.reserve(num_spheres / chunk_size + 1);
            for ((positions_chunk, radii_chunk), colours_chunk) in positions.chunks(chunk_size)
                .zip(radii.chunks(chunk_size))
                .zip(colours.chunks(chunk_size)) {
                self.chunks.push(Chunk {
                    positions: try!(Texture1d::with_format(window.facade(),
                                                           positions_chunk,
                                                           UncompressedFloatFormat::F32F32F32,
                                                           MipmapsOption::NoMipmap)
                        .chain_err(|| "Failed to build positions texture.")),
                    radii: try!(Texture1d::with_format(window.facade(),
                                                       radii_chunk,
                                                       UncompressedFloatFormat::F32,
                                                       MipmapsOption::NoMipmap)
                        .chain_err(|| "Failed to build radii texture.")),
                    colours: try!(Texture1d::with_format(window.facade(),
                                                         colours_chunk,
                                                         UncompressedFloatFormat::F32F32F32,
                                                         MipmapsOption::NoMipmap)
                        .chain_err(|| "Failed to build colours texture.")),
                    size: positions_chunk.len(),
                })
            }
        }

        let modelview = camera.modelview();
        let projection = camera.projection();
        let transformed_lights = self.settings.lights.transformed(&modelview);

        for chunk in &self.chunks {
            let uniforms = uniform! {
                u_modelview: &modelview,
                u_projection: projection,
                u_key_light_direction: &transformed_lights.key.direction,
                u_key_light_colour: &transformed_lights.key.colour,
                u_fill_light_direction: &transformed_lights.fill.direction,
                u_fill_light_colour: &transformed_lights.fill.colour,
                u_back_light_direction: &transformed_lights.back.direction,
                u_back_light_colour: &transformed_lights.back.colour,
                u_ambient_colour: &transformed_lights.ambient,
                u_positions: chunk.positions.sampled()
                                      .magnify_filter(MagnifySamplerFilter::Nearest)
                                      .minify_filter(MinifySamplerFilter::Nearest)
                                      .wrap_function(SamplerWrapFunction::Clamp),
                u_radii: chunk.radii.sampled()
                              .magnify_filter(MagnifySamplerFilter::Nearest)
                              .minify_filter(MinifySamplerFilter::Nearest)
                              .wrap_function(SamplerWrapFunction::Clamp),
                u_colours: chunk.colours.sampled()
                                  .magnify_filter(MagnifySamplerFilter::Nearest)
                                  .minify_filter(MinifySamplerFilter::Nearest)
                                  .wrap_function(SamplerWrapFunction::Clamp),
            };

            try!(frame.draw(EmptyVertexAttributes { len: chunk.size * 6 },
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
