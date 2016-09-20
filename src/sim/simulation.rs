use idcontain::{IdVec, Id};
use math::{Vec3f, Vector, Vec4f, Aabb, Sphere, ContactInfo};
use num::Zero;
use std::f32::consts;
use super::bvh::Bvh;
use super::frame_timers::{FrameTimers, FrameTimerId};
use rayon::prelude::*;
use std::cell::RefCell;
use super::mesh_renderer::Mesh;


#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EntityId(Id<ComponentIndexes>);

pub struct Simulation {
    entities: IdVec<ComponentIndexes>,

    bvh_timer: FrameTimerId,
    forces_timer: FrameTimerId,
    positions_timer: FrameTimerId,

    sphere_bvh: Bvh,

    world_mesh: Mesh,
    world_bvh: Bvh,
    world_triangles: Vec<Triangle>,

    timestep: f32,
    colours: Vec<Vec3f>,
    forces: Vec<Vec3f>,
    masses: Vec<f32>,
    positions: Vec<Vec3f>,
    radii: Vec<f32>,
    reverse_lookup: Vec<EntityId>,
    velocities: Vec<Vec3f>,

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

struct ComponentIndexes {
    base: u32,
}


impl Simulation {
    pub fn with_capacity(frame_timers: &mut FrameTimers, capacity: u32, world_mesh: Mesh) -> Self {
        let capacity = capacity as usize;

        let mut world_triangles = Vec::with_capacity(world_mesh.triangles.len());
        let world_bvh = {
            let mut bvh = Bvh::new(2);
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
            sphere_bvh: Bvh::new(6),

            world_mesh: world_mesh,
            world_bvh: world_bvh,
            world_triangles: world_triangles,

            bvh_timer: frame_timers.new_stopped("sim.bvh"),
            forces_timer: frame_timers.new_stopped("sim.frc"),
            positions_timer: frame_timers.new_stopped("sim.pos"),

            timestep: 1. / 300.,

            colours: Vec::with_capacity(capacity),
            forces: Vec::with_capacity(capacity),
            masses: Vec::with_capacity(capacity),
            positions: Vec::with_capacity(capacity),
            radii: Vec::with_capacity(capacity),
            reverse_lookup: Vec::with_capacity(capacity),
            velocities: Vec::with_capacity(capacity),

            explode: false,
        }
    }

    pub fn add(&mut self, entity: NewEntity) -> EntityId {
        let base_index = self.len() as u32;
        let id = EntityId(self.entities.insert(ComponentIndexes { base: base_index }));

        self.colours.push(entity.colour);
        self.forces.push(Vec3f::zero());
        self.masses.push(entity.mass);
        self.positions.push(entity.position);
        self.radii.push(entity.radius);
        self.reverse_lookup.push(id);
        self.velocities.push(entity.velocity);
        id
    }

    pub fn remove(&mut self, id: EntityId) -> bool {
        if let Some(ComponentIndexes { base: u32_index }) = self.entities.remove(id.0) {
            let swap_index = self.len() - 1;
            let index = u32_index as usize;
            if swap_index == index {
                self.colours.pop();
                self.forces.pop();
                self.masses.pop();
                self.positions.pop();
                self.radii.pop();
                self.reverse_lookup.pop();
                self.velocities.pop();
            } else {
                self.colours.swap_remove(index);
                self.forces.swap_remove(index);
                self.masses.swap_remove(index);
                self.positions.swap_remove(index);
                self.radii.swap_remove(index);
                self.reverse_lookup.swap_remove(index);
                self.velocities.swap_remove(index);

                self.entities[self.reverse_lookup[index].0].base = u32_index;
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
        self.zero_forces();
        timers.start(self.forces_timer);
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
    }

    fn compute_forces(&mut self) {
        let len = self.len();
        let forces = &mut self.forces[..];
        let world_triangles = &self.world_triangles[..];
        let positions = &self.positions[..];
        let velocities = &self.velocities[..];
        let radii = &self.radii[..];
        let sphere_bvh = &self.sphere_bvh;
        let world_bvh = &self.world_bvh;
        let explode = self.explode;
        let timestep = self.timestep;


        assert!(forces.len() == len);
        assert!(positions.len() == len);
        assert!(velocities.len() == len);
        assert!(radii.len() == len);

        forces.par_iter_mut()
            .enumerate()
            .zip(positions)
            .zip(velocities)
            .zip(radii)
            .for_each(|((((i_body, force), &position), &velocity), &radius)| {
                let speed = velocity.norm();
                let drag = speed * consts::PI * radius * radius * DRAG_COEFFICIENT;
                *force -= velocity * drag;
                *force += Vec3f::new(0.0, -GRAVITY, 0.0);

                if explode {
                    let direction = position - Vec3f::new(-5.0, 2.0, 5.0);
                    let distance_squared = direction.squared_norm().max(1.0);
                    let distance = distance_squared.sqrt();
                    *force += direction * EXPLODE / (distance_squared * distance);
                }

                THREAD_INTERSECTION_STACK.with(|intersection_stack| {
                    let mut intersection_stack = intersection_stack.borrow_mut();

                    for j_body in
                        sphere_bvh.intersect_sphere(&mut intersection_stack, position, radius) {
                        if j_body == i_body {
                            continue;
                        }
                        let other_position = positions[j_body];
                        let other_radius = radii[j_body];
                        let other_velocity = velocities[j_body];

                        let direction = position - other_position;
                        let min_distance = radius + other_radius;
                        let squared_distance = direction.squared_norm();
                        if squared_distance < min_distance * min_distance {
                            let distance = squared_distance.sqrt() + 1e-8;
                            let correction = (min_distance / distance - 1.0) * COMPRESS * 0.5;

                            let relative = velocity - other_velocity;
                            let normal_dot = relative.dot(&direction) / distance;
                            if normal_dot < 0.0 {
                                let normal_dot = normal_dot.max(-0.5 / (timestep * 0.5));
                                *force += relative * normal_dot * 1.0;
                            }

                            *force += direction * correction;
                        }
                    }
                });
            });
    }

    fn velocities_halfstep(&mut self) {
        let mut velocities = &mut self.velocities[..];
        let mut forces = &mut self.forces[..];
        let mut masses = &mut self.masses[..];

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
            forces = &mut deborrow(forces)[4..];
            masses = &mut deborrow(masses)[4..];
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
                THREAD_INTERSECTION_STACK.with(|intersection_stack| {
                    let mut timestep = timestep;
                    while timestep > 0.0 {
                        let speed = velocity.norm();
                        let sphere = Sphere::new(*position + *velocity * (timestep * 0.5),
                                                 speed * timestep * 0.51);
                        let (time, index) =
                            world_bvh.intersect_sphere(&mut intersection_stack.borrow_mut(),
                                                  sphere.center,
                                                  sphere.radius)
                                .map(|index| (index, &world_triangles[index]))
                                .flat_map(|(index, triangle)| {
                                    ray_triangle(position, velocity, &triangle)
                                        .map(|time| (time, index))
                                })
                                .fold((timestep, !0), |x, y| if x <= y { x } else { y });
                        *position += *velocity * (time - 1e-8);
                        timestep -= time;
                        if index != !0 {
                            let normal = world_triangles[index].normal;
                            *velocity -= normal * (velocity.dot(&normal) * 1.5);
                        }
                    }
                })
            });
    }

    fn remove_outliers(&mut self) {
        let mut removals = Vec::new();
        for (i_position, position) in self.positions.iter_mut().enumerate() {
            if position.squared_norm() > 1000.0 * 1000.0 {
                removals.push(self.reverse_lookup[i_position]);
            }
        }

        for removal in removals {
            self.remove(removal);
            info!("Remove entity {:?}", removal);
        }
    }
}

fn deborrow<T>(value: T) -> T {
    value
}


fn intersect_sphere_line(center: &Vec3f, radius: f32, p1: &Vec3f, p2: &Vec3f) -> Option<f32> {
    let edge = *p2 - *p1;
    let a = edge.squared_norm();
    let b = 2.0 * edge.dot(&(*p1 - *center));
    let c = center.squared_norm() + p1.squared_norm() - 2.0 * center.dot(p1) - radius * radius;
    lowest_quadratic_root(a, b, c)
}

fn lowest_quadratic_root(a: f32, b: f32, c: f32) -> Option<f32> {
    let i = b * b - 4.0 * a * c;
    if i < 0.0 {
        None
    } else {
        let i = i.sqrt();
        let a2 = 2.0 * a;
        let i1 = (-b + i) / a2;
        let i2 = (-b - i) / a2;
        if i1 < i2 { Some(i1) } else { Some(i2) }
    }
}

struct Triangle {
    vertices: [Vec3f; 3],
    normal: Vec3f,
    intercept: f32,

    u: Vec3f,
    v: Vec3f,
}

impl Triangle {
    fn new(vertices: &[Vec3f; 3]) -> Self {
        let u = vertices[1] - vertices[0];
        let v = vertices[2] - vertices[0];
        let normal = u.cross(v).normalized();
        let intercept = -normal.dot(&vertices[0]);

        Triangle {
            vertices: *vertices,
            normal: normal,
            intercept: intercept,

            u: u,
            v: v,
        }
    }
}

const DRAG_COEFFICIENT: f32 = 1.0;
const COMPRESS: f32 = 1000.0;
const EXPLODE: f32 = 500.0;
const GRAVITY: f32 = 20.0;

thread_local! {
    static THREAD_INTERSECTION_STACK: RefCell<Vec<usize>> = RefCell::new(Vec::with_capacity(128));
}

fn ray_triangle(origin: &Vec3f, direction: &Vec3f, triangle: &Triangle) -> Option<f32> {
    let &Triangle { ref vertices, ref u, ref v, .. } = triangle;
    let pvec = direction.cross(*v);
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

    let qvec = tvec.cross(*u);
    let beta = direction.dot(&qvec) * inv_det;
    if beta < -1e-5 || alpha + beta > 1.0 + 1e-5 {
        return None;
    }

    let t = v.dot(&qvec) * inv_det;
    if t > -1e-2 { Some(t) } else { None }
}
