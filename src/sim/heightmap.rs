use math::{Vec3f, vec3};
use sdl2_image::LoadSurface;
use sdl2::surface::Surface;
use sdl2::pixels::PixelFormatEnum;
use std::path::Path;
use super::errors::{Result, ResultExt};
use gfx::errors::SdlError;
use super::mesh_renderer::{Mesh, MeshVertex, Triangle};
use num::Zero;
use std::mem;

const PIXEL_SIZE: usize = 4;

pub struct Heightmap {
    width: usize,
    height: usize,
    data: Vec<f32>,
}

impl Heightmap {
    pub fn read<P: AsRef<Path>>(path: P, resolution: f32) -> Result<Self> {
        Ok(rescale_heightmap(try!(load_heightmap(path.as_ref())), resolution))
    }

    pub fn build_mesh(&self, centre: Vec3f, size: Vec3f) -> Mesh {
        let Heightmap { width, height, ref data } = *self;
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        vertices.reserve(width * height);
        triangles.reserve((width - 1) * (height - 1) * 2);

        for y in 0..height {
            for x in 0..width {
                vertices.push(MeshVertex {
                    position: vec3(x as f32 / (width - 1) as f32 * 2.0 - 1.0,
                                   data[y * width + x],
                                   y as f32 / (height - 1) as f32 * 2.0 - 1.0) *
                              size + centre,
                    normal: Vec3f::zero(),
                });
            }
        }

        for y in 0..height as u32 - 1 {
            for x in 0..width as u32 - 1 {
                let top_left = y * width as u32 + x;
                let top_right = top_left + 1;
                let bottom_left = top_left + width as u32;
                let bottom_right = bottom_left + 1;
                if (x + y) % 2 == 0 {
                    triangles.push(Triangle([top_right, top_left, bottom_right]));
                    triangles.push(Triangle([bottom_right, top_left, bottom_left]));
                } else {
                    triangles.push(Triangle([bottom_left, bottom_right, top_right]));
                    triangles.push(Triangle([top_right, top_left, bottom_left]));
                }
            }
        }

        let mut mesh = Mesh {
            vertices: vertices,
            triangles: triangles,
        };
        mesh.set_smooth_normals();
        mesh
    }
}

fn load_heightmap(path: &Path) -> Result<Heightmap> {
    let raw_surface = try!(Surface::from_file(path)
        .map_err(SdlError)
        .chain_err(|| format!("Cannot read heightmap image from {:?}.", path)));
    let width = raw_surface.width();
    let height = raw_surface.height();
    let mut surface = try!(Surface::new(width, height, PixelFormatEnum::ARGB8888)
        .map_err(SdlError)
        .chain_err(|| {
            format!("Cannot allocate converted surface for heightmap at {:?}",
                    path)
        }));
    try!(raw_surface.blit(None, &mut surface, None)
        .map_err(SdlError)
        .chain_err(|| format!("Surface conversion failed for heightmap at {:?}", path)));

    let (width, height, pitch) =
        (surface.width() as usize, surface.height() as usize, surface.pitch() as usize);

    let mut data = Vec::new();
    if width == 0 || height == 0 {
        warn!("Heightmap {:?} has zero dimenion ({}x{}).",
              path,
              width,
              height);
    } else {
        data.resize(width * height, 0.0);
        surface.with_lock(|bytes| {
            assert!(bytes.len() >= (height - 1) * pitch + width * PIXEL_SIZE);
            for y in 0..height {
                for x in 0..width {
                    data[y * width + x] = bytes[y * pitch + x * PIXEL_SIZE + 1] as f32 / 255.0;
                }
            }
        });
    }

    Ok(Heightmap {
        data: data,
        width: width,
        height: height,
    })
}


fn rescale_heightmap(input: Heightmap, mut scale: f32) -> Heightmap {
    if scale == 1.0 {
        return input;
    }

    let mut source = input;
    let mut destination = Heightmap {
        data: vec![0.0; source.data.len()],
        width: source.width,
        height: source.height,
    };

    let coords_iter = |heightmap: &Heightmap| {
        let Heightmap { width, height, .. } = *heightmap;
        (0..height).flat_map(move |y| (0..width).map(move |x| (x, y)))
    };

    while scale <= 0.5 {
        destination.width = source.width / 2;
        destination.height = source.height / 2;
        destination.data.truncate(destination.width * destination.height);
        scale *= 2.0;
        for (x, y) in coords_iter(&destination) {
            let top_left = y * 2 * source.width + x * 2;
            let top_right = top_left + 1;
            let bottom_left = top_left + source.width;
            let bottom_right = bottom_left + 1;
            destination.data[y * destination.width + x] =
                (source.data[top_left] + source.data[top_right] + source.data[bottom_left] +
                 source.data[bottom_right]) * 0.25;
        }
        mem::swap(&mut source, &mut destination);
    }

    let inverse_scale = 1.0 / scale;
    destination.width = (source.width as f32 * scale) as usize;
    destination.height = (source.height as f32 * scale) as usize;
    destination.data.resize(destination.width * destination.height, 0.0);
    if destination.width == source.width && destination.height == source.height {
        return source;
    }
    for (x, y) in coords_iter(&destination) {
        let source_x = inverse_scale * (x as f32);
        let source_y = inverse_scale * (y as f32);
        let frac_x = source_x - source_x.floor();
        let frac_y = source_y - source_y.floor();
        let (source_x, source_y) = (source_x as usize, source_y as usize);

        let top_left = source_y * source.width + source_x;
        let top_right = top_left + 1;
        let bottom_left = top_left + source.width;
        let bottom_right = bottom_left + 1;

        let top = if source_x == source.width - 1 {
            source.data[top_left]
        } else {
            source.data[top_left] * (1.0 - frac_x) + source.data[top_right] * frac_x
        };
        let bottom = if source_y == source.height - 1 {
            top
        } else if source_x == source.width - 1 {
            source.data[bottom_left]
        } else {
            source.data[bottom_left] * (1.0 - frac_x) + source.data[bottom_right] * frac_x
        };

        destination.data[y * destination.width + x] = top * (1.0 - frac_y) + bottom * frac_y;
    }

    destination
}
