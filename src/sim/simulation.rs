use idcontain::{IdVec, Id};
use math::{vec3, Vec3f, Vector, Aabb};
use num::Zero;
use std::f32::consts;
use super::bvh::{Bvh, Options as BvhOptions, MinLeaves, Two, Four, Six, Sixteen,
                 BinnedSahPartition, TotalAabbLimit, CentroidAabbLimit};
use super::frame_timers::{FrameTimers, FrameTimerId};
use rayon::prelude::*;
use super::mesh_renderer::Mesh;
use parking_lot::Mutex;


type SphereBvh = Bvh<BvhOptions<MinLeaves<Six>, BinnedSahPartition<Four, TotalAabbLimit>>>;
type WorldBvh = Bvh<BvhOptions<MinLeaves<Two>, BinnedSahPartition<Sixteen, CentroidAabbLimit>>>;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EntityId(Id<ComponentIndex>);

pub struct Simulation {
    entities: IdVec<ComponentIndex>,

    bvh_timer: FrameTimerId,
    forces_timer: FrameTimerId,
    positions_timer: FrameTimerId,

    sphere_bvh: SphereBvh,

    world_bvh: WorldBvh,
    world_triangles: Vec<Triangle>,

    timestep: f32,
    colours: Vec<Vec3f>,
    forces: Vec<Vec3f>,
    masses: Vec<f32>,
    positions: Vec<Vec3f>,
    radii: Vec<f32>,
    reverse_lookup: Vec<EntityId>,
    velocities: Vec<Vec3f>,

    other_forces: Vec<Mutex<Vec3f>>,

    explode: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct NewEntity {
    pub position: Vec3f,
    pub velocity: Vec3f,
    pub radius: f32,
    pub mass: f32,
    pub colour: Vec3f,
}

struct ComponentIndex {
    index: u32,
}

impl Simulation {
    pub fn with_capacity(frame_timers: &mut FrameTimers, capacity: u32, world_mesh: &Mesh) -> Self {
        let capacity = capacity as usize;

        let mut world_triangles = Vec::with_capacity(world_mesh.triangles.len());
        let world_bvh = {
            let mut bvh = Bvh::new();
            bvh.rebuild(world_mesh.triangles.iter().map(|triangle| {
                Aabb::of_points(&[world_mesh.vertices[triangle[0] as usize].position,
                                  world_mesh.vertices[triangle[1] as usize].position,
                                  world_mesh.vertices[triangle[2] as usize].position])
            }));
            bvh
        };

        world_triangles.extend(world_mesh.triangles.iter().map(|triangle| {
            Triangle::new(&[world_mesh.vertices[triangle[0] as usize].position,
                            world_mesh.vertices[triangle[1] as usize].position,
                            world_mesh.vertices[triangle[2] as usize].position])
        }));

        Simulation {
            entities: IdVec::with_capacity(capacity),
            sphere_bvh: Bvh::new(),

            world_bvh: world_bvh,
            world_triangles: world_triangles,

            bvh_timer: frame_timers.new_stopped("sim.bvh"),
            forces_timer: frame_timers.new_stopped("sim.frc"),
            positions_timer: frame_timers.new_stopped("sim.pos"),

            timestep: 1. / 300.,

            colours: Vec::with_capacity(capacity),
            forces: Vec::with_capacity(capacity),
            other_forces: Vec::with_capacity(capacity),
            masses: Vec::with_capacity(capacity),
            positions: Vec::with_capacity(capacity),
            radii: Vec::with_capacity(capacity),
            reverse_lookup: Vec::with_capacity(capacity),
            velocities: Vec::with_capacity(capacity),

            explode: false,
        }
    }

    pub fn add(&mut self, entity: NewEntity) -> EntityId {
        let index_index = self.len() as u32;
        let id = EntityId(self.entities.insert(ComponentIndex { index: index_index }));

        self.colours.push(entity.colour);
        self.forces.push(Vec3f::zero());
        self.other_forces.push(Mutex::new(Vec3f::zero()));
        self.masses.push(entity.mass);
        self.positions.push(entity.position);
        self.radii.push(entity.radius);
        self.reverse_lookup.push(id);
        self.velocities.push(entity.velocity);
        id
    }

    pub fn remove(&mut self, id: EntityId) -> bool {
        if let Some(ComponentIndex { index }) = self.entities.remove(id.0) {
            let index = index as usize;
            self.colours.swap_remove(index);
            self.forces.swap_remove(index);
            self.other_forces.swap_remove(index);
            self.masses.swap_remove(index);
            self.positions.swap_remove(index);
            self.radii.swap_remove(index);
            self.reverse_lookup.swap_remove(index);
            self.velocities.swap_remove(index);
            if index < self.reverse_lookup.len() {
                self.entities[self.reverse_lookup[index].0].index = index as u32;
            }
            true
        } else {
            false
        }
    }

    pub fn colours(&self) -> &[Vec3f] {
        &self.colours
    }

    pub fn positions(&self) -> &[Vec3f] {
        &self.positions
    }

    pub fn radii(&self) -> &[f32] {
        &self.radii
    }

    pub fn explode(&mut self) {
        self.explode = true;
    }

    pub fn update(&mut self, timers: &mut FrameTimers, _delta_time: f32) {
        self.velocities_halfstep();
        timers.start(self.positions_timer);
        self.update_positions();
        timers.stop(self.positions_timer);
        self.remove_outliers();
        timers.start(self.bvh_timer);
        self.update_bvh();
        timers.stop(self.bvh_timer);
        timers.start(self.forces_timer);
        self.zero_forces();
        self.compute_forces();
        timers.stop(self.forces_timer);
        self.velocities_halfstep();

        self.explode = false;
    }

    pub fn update_bvh(&mut self) {
        self.sphere_bvh
            .rebuild(self.positions
                .iter()
                .zip(self.radii.iter())
                .map(|(p, &r)| Aabb::of_sphere(p, r)));
    }

    pub fn len(&self) -> usize {
        self.entities.len()
    }

    fn zero_forces(&mut self) {
        self.forces.clear();
        self.forces.resize(self.positions.len(), Vec3f::zero());
        for force in &mut self.other_forces {
            *force.get_mut() = Vec3f::zero();
        }
    }

    fn compute_forces(&mut self) {
        let Simulation { ref mut forces,
                         ref mut other_forces,
                         ref positions,
                         ref velocities,
                         ref radii,
                         ref sphere_bvh,
                         timestep,
                         .. } = *self;
        let len = positions.len();
        assert!(forces.len() == len);
        assert!(velocities.len() == len);
        assert!(radii.len() == len);
        assert!(other_forces.len() == len);

        forces.par_iter_mut()
            .enumerate()
            .zip(positions)
            .zip(velocities)
            .zip(radii)
            .for_each(|((((i_body, force), position), velocity), &radius)| {
                let speed = velocity.norm();
                let drag = speed * consts::PI * radius * radius * DRAG_COEFFICIENT;
                *force -= *velocity * drag;
                *force += vec3(0.0, -GRAVITY, 0.0);

                sphere_bvh.on_sphere_intersection(position, radius, i_body + 1, |j_body| {
                    let other_position = &positions[j_body];
                    let other_radius = radii[j_body];

                    let direction = *position - *other_position;
                    let min_distance = radius + other_radius;
                    let squared_distance = direction.squared_norm();
                    if squared_distance < min_distance * min_distance {
                        let other_velocity = &velocities[j_body];
                        let distance = squared_distance.sqrt() + 1e-8;
                        let correction = (min_distance / distance - 1.0) * COMPRESS * 0.5;

                        let relative = *velocity - *other_velocity;
                        let normal_dot = relative.dot(&direction) / distance;
                        if normal_dot < 0.0 {
                            let normal_dot = normal_dot.max(-0.5 / (timestep * 0.5));
                            *force += relative * normal_dot * 1.0;
                        }

                        let new_force = direction * correction;
                        *force += new_force;
                        *(&other_forces[j_body]).lock() -= new_force;
                    }
                });
            });

        forces.par_iter_mut().zip(other_forces.par_iter_mut()).for_each(|(force, other_force)| {
            *force += *other_force.get_mut();
        });
    }

    fn velocities_halfstep(&mut self) {
        let mut velocities = &mut self.velocities[..];
        let mut forces = &self.forces[..];
        let mut masses = &self.masses[..];

        assert!(velocities.len() == forces.len());
        assert!(velocities.len() == masses.len());

        let half_timestep = self.timestep * 0.5;
        let update = |velocity: &mut Vec3f, force: Vec3f, mass: f32| {
            *velocity += force * (half_timestep / mass);
        };

        while velocities.len() > 4 {
            update(&mut velocities[0], forces[0], masses[0]);
            update(&mut velocities[1], forces[1], masses[1]);
            update(&mut velocities[2], forces[2], masses[2]);
            update(&mut velocities[3], forces[3], masses[3]);

            velocities = &mut deborrow(velocities)[4..];
            forces = &forces[4..];
            masses = &masses[4..];
        }

        for i in 0..velocities.len() {
            update(&mut velocities[i], forces[i], masses[i]);
        }
    }

    fn update_positions(&mut self) {
        let Simulation { ref mut positions,
                         ref mut velocities,
                         ref world_triangles,
                         ref world_bvh,
                         timestep,
                         .. } = *self;

        assert!(positions.len() == velocities.len());
        positions.par_iter_mut()
            .zip(velocities)
            .for_each(|(position, velocity)| {
                let mut timestep = timestep;
                for _ in 0..40 {
                    let speed = velocity.norm();
                    let probe_position = *position + *velocity * (timestep * 0.5);
                    let probe_radius = speed * timestep * 0.51;
                    let mut first_time = timestep;
                    let mut first_index = !0;
                    world_bvh.on_sphere_intersection(&probe_position, probe_radius, 0, |index| {
                        if let Some(time) = ray_triangle(position,
                                                         velocity,
                                                         &world_triangles[index].vertices) {
                            if time < first_time {
                                first_time = time;
                                first_index = index;
                            }
                        }
                    });
                    *position += *velocity * first_time;
                    timestep -= first_time;
                    if timestep <= 1e-10 {
                        break;
                    }
                    let normal = world_triangles[first_index].normal;
                    *velocity -= normal * (velocity.dot(&normal) * 1.5);
                }
            });
    }

    fn remove_outliers(&mut self) {
        assert_eq!(self.positions.len(), self.reverse_lookup.len());
        for i_position in (0..self.positions.len()).rev() {
            if self.positions[i_position][1] < -5.0 {
                let entity_id = self.reverse_lookup[i_position];
                self.remove(entity_id);
            }
        }
    }
}


fn deborrow<T>(value: T) -> T {
    value
}

struct Triangle {
    vertices: [Vec3f; 3],
    normal: Vec3f,
}

impl Triangle {
    fn new(vertices: &[Vec3f; 3]) -> Self {
        let u = vertices[1] - vertices[0];
        let v = vertices[2] - vertices[0];
        let normal = u.cross(&v).normalized();

        Triangle {
            vertices: *vertices,
            normal: normal,
        }
    }
}

const DRAG_COEFFICIENT: f32 = 1.0;
const COMPRESS: f32 = 3000.0;
const GRAVITY: f32 = 20.0;

#[inline]
fn ray_triangle(origin: &Vec3f, direction: &Vec3f, vertices: &[Vec3f]) -> Option<f32> {
    let u = vertices[1] - vertices[0];
    let v = vertices[2] - vertices[0];
    let pvec = direction.cross(&v);
    let det = u.dot(&pvec);

    if det < 1e-10 {
        return None;
    }

    let inv_det = 1.0 / det;
    let tvec = *origin - vertices[0];
    let alpha = tvec.dot(&pvec) * inv_det;
    if alpha < -1e-5 || alpha > 1.0 + 1e-5 {
        return None;
    }

    let qvec = tvec.cross(&u);
    let beta = direction.dot(&qvec) * inv_det;
    if beta < -1e-5 || alpha + beta > 1.0 + 1e-5 {
        return None;
    }

    let t = v.dot(&qvec) * inv_det;
    if t > -1e-2 { Some(t) } else { None }
}
