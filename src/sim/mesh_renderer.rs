use gfx::{Window, Camera};
use glium::index::PrimitiveType;
use glium::{VertexBuffer, IndexBuffer, Frame, Surface, Program, DrawParameters,
            BackfaceCullingMode, Depth, DepthTest};
use idcontain::{IdVec, Id};
use math::{Vec3f, Mat4, Vector};
use super::errors::{Result, ChainErr};
use super::lights::Lights;
use std::ops::{Deref, DerefMut};

#[repr(C)]
#[derive(Copy, Clone)]
pub struct Triangle(pub [u32; 3]);

impl Deref for Triangle {
    type Target = [u32; 3];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Triangle {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct MeshVertex {
    pub position: Vec3f,
    pub normal: Vec3f,
}

#[derive(Clone)]
pub struct Mesh {
    pub vertices: Vec<MeshVertex>,
    pub triangles: Vec<Triangle>,
}

impl Mesh {
    pub fn set_smooth_normals(&mut self) {
        for triangle in &self.triangles {
            let positions = [self.vertices[triangle[0] as usize].position,
                             self.vertices[triangle[1] as usize].position,
                             self.vertices[triangle[2] as usize].position];
            let normal = (positions[1] - positions[0])
                .cross(positions[2] - positions[0])
                .normalized();
            self.vertices[triangle[0] as usize].normal += normal;
            self.vertices[triangle[1] as usize].normal += normal;
            self.vertices[triangle[2] as usize].normal += normal;
        }

        for vertex in &mut self.vertices {
            vertex.normal.normalize();
        }
    }
}


#[derive(Copy, Clone)]
pub struct MeshRef<'a> {
    pub vertices: &'a [MeshVertex],
    pub triangles: &'a [Triangle],
}

impl<'a> Into<MeshRef<'a>> for &'a Mesh {
    fn into(self) -> MeshRef<'a> {
        MeshRef {
            vertices: &self.vertices,
            triangles: &self.triangles,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct MeshId(Id<MeshBuffers>);

pub struct MeshRenderer {
    meshes: IdVec<MeshBuffers>,
    program: Program,
    draw_parameters: DrawParameters<'static>,
    settings: Settings,

    index_scratch: Vec<u32>,
}

#[derive(Default)]
pub struct Settings {
    pub lights: Lights,
}

#[derive(Copy, Clone)]
pub struct MeshList<'a> {
    pub ids: &'a [MeshId],
    pub colours: &'a [Vec3f],
    pub transforms: &'a [Mat4],
}

impl MeshRenderer {
    pub fn new(window: &Window, settings: Settings) -> Result<Self> {
        Ok(MeshRenderer {
            meshes: IdVec::with_capacity(64),
            index_scratch: Vec::with_capacity(1024),
            program: try!(window.program(VERTEX_SHADER, FRAGMENT_SHADER)
                .chain_err(|| "Failed to build program for MeshRenderer.")),
            draw_parameters: DrawParameters {
                depth: Depth {
                    test: DepthTest::IfLess,
                    write: true,
                    ..Depth::default()
                },
                backface_culling: BackfaceCullingMode::CullClockwise,
                ..DrawParameters::default()
            },
            settings: settings,
        })
    }

    pub fn add<'a, M>(&mut self, window: &Window, mesh: M) -> Result<MeshId>
        where M: Into<MeshRef<'a>>
    {
        let mesh_ref = mesh.into();
        self.index_scratch.clear();
        self.index_scratch.reserve(mesh_ref.triangles.len() * 3);
        self.index_scratch.extend(mesh_ref.triangles.iter().flat_map(|triangle| triangle.iter()));

        Ok(MeshId(self.meshes.insert(MeshBuffers {
            vertices: try!(VertexBuffer::immutable(window.facade(), mesh_ref.vertices)
                .chain_err(|| "Failed to create vertex buffer for SimpleMesh.")),
            triangles: try!(IndexBuffer::immutable(window.facade(),
                                                   PrimitiveType::TrianglesList,
                                                   &self.index_scratch)
                .chain_err(|| "Failed to create index buffer for SimpleMesh.")),
        })))
    }

    // pub fn remove(&mut self, mesh: MeshId) -> bool {
    //    self.meshes.remove(mesh.0).is_some()
    // }

    pub fn render<'a>(&self,
                      _window: &Window,
                      camera: &Camera,
                      frame: &mut Frame,
                      mesh_list: MeshList<'a>)
                      -> Result<()> {
        let modelview = camera.modelview();
        let projection = camera.projection();
        let transformed_lights = self.settings.lights.transformed(&modelview);
        for ((mesh_id, transform), colour) in mesh_list.ids
            .iter()
            .zip(mesh_list.transforms)
            .zip(mesh_list.colours) {
            let transform = modelview * *transform;
            let normal_matrix = match transform.inversed().map(|x| x.transposed()) {
                Some(matrix) => matrix,
                None => {
                    error!("Non-invertible modelview matrix: {:?}", modelview);
                    modelview
                }
            };
            let uniforms = uniform! {
                    u_modelview: transform,
                    u_normal_matrix: &normal_matrix,
                    u_projection: projection,
                    u_colour: colour,
                    u_key_light_direction: &transformed_lights.key.direction,
                    u_key_light_colour: &transformed_lights.key.colour,
                    u_fill_light_direction: &transformed_lights.fill.direction,
                    u_fill_light_colour: &transformed_lights.fill.colour,
                    u_back_light_direction: &transformed_lights.back.direction,
                    u_back_light_colour: &transformed_lights.back.colour,
                    u_ambient_colour: &transformed_lights.ambient,
            };
            if let Some(mesh) = self.meshes.get(mesh_id.0) {
                try!(frame.draw(&mesh.vertices,
                          &mesh.triangles,
                          &self.program,
                          &uniforms,
                          &self.draw_parameters)
                    .chain_err(|| "Failed to draw mesh."));
            } else {
                error!("Attempting to render missing mesh: {:?}", mesh_id);
            }
        }
        Ok(())
    }
}

struct MeshBuffers {
    vertices: VertexBuffer<MeshVertex>,
    triangles: IndexBuffer<u32>,
}


implement_vertex!(MeshVertex, position, normal);

const VERTEX_SHADER: &'static str = "src/shaders/simple_mesh.vert";
const FRAGMENT_SHADER: &'static str = "src/shaders/simple_mesh.frag";
