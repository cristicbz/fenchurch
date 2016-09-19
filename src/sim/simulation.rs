use idcontain::{IdVec, Id};
use math::{Vec3f, Vector, Vec4f, Aabb};
use num::Zero;
use std::f32::consts;
use super::bvh::Bvh;
use super::frame_timers::{FrameTimers, FrameTimerId};
use rayon::prelude::*;
use std::cell::RefCell;
use super::mesh_renderer::Mesh;

const DRAG_COEFFICIENT: f32 = 1.0;
const COMPRESS: f32 = 1000.0;
const EXPLODE: f32 = 500.0;
const GRAVITY: f32 = 20.0;

struct Triangle {
    normal: Vec3f,
    intercept: f32,
    vertices: [Vec3f; 3],
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EntityId(Id<ComponentIndexes>);

pub struct Simulation {
    entities: IdVec<ComponentIndexes>,

    bvh_timer: FrameTimerId,
    forces_timer: FrameTimerId,

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
            let mut bvh = Bvh::new();
            bvh.rebuild(world_mesh.triangles.iter().map(|triangle| {
                Aabb::of_points(&[world_mesh.vertices[triangle[0] as usize].position,
                                  world_mesh.vertices[triangle[1] as usize].position,
                                  world_mesh.vertices[triangle[2] as usize].position])
            }));
            bvh
        };

        world_triangles.extend(world_mesh.triangles.iter().map(|triangle| {
            let vertices = [world_mesh.vertices[triangle[0] as usize].position,
                            world_mesh.vertices[triangle[1] as usize].position,
                            world_mesh.vertices[triangle[2] as usize].position];
            let edges = [vertices[1] - vertices[0], vertices[2] - vertices[0]];
            let normal = edges[0].cross(edges[1]).normalized();
            let intercept = -normal.dot(&vertices[0]);
            Triangle {
                normal: normal,
                intercept: intercept,
                vertices: vertices,
            }
        }));

        Simulation {
            entities: IdVec::with_capacity(capacity),
            sphere_bvh: Bvh::new(),

            world_mesh: world_mesh,
            world_bvh: world_bvh,
            world_triangles: world_triangles,

            bvh_timer: frame_timers.new_stopped("sim.bvh"),
            forces_timer: frame_timers.new_stopped("sim.frc"),

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
        self.update_positions();
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

                thread_local! {
                    static THREAD_INTERSECTION_STACK: RefCell<Vec<usize>> =
                        RefCell::new(Vec::with_capacity(128));
                };
                THREAD_INTERSECTION_STACK.with(|intersection_stack| {
                    let mut intersection_stack = intersection_stack.borrow_mut();

                    for triangle_index in
                        world_bvh.intersect_sphere(&mut intersection_stack, position, radius) {
                        let Triangle { mut normal, intercept, ref vertices } =
                            world_triangles[triangle_index];

                        let mut min_distance = radius;
                        let mut contact_normal = None;

                        // Sphere against plane.
                        let signed_plane_distance = normal.dot(&position) + intercept;
                        let plane_distance = signed_plane_distance.abs();
                        if plane_distance >= radius {
                            continue;
                        }
                        let centre_on_plane = position - normal * signed_plane_distance;
                        if is_point_inside_triangle(vertices, &centre_on_plane) {
                            min_distance = plane_distance;
                            contact_normal = Some(normal);
                        }

                        // Sphere against vertices.
                        for vertex in vertices {
                            let to_centre = position - *vertex;
                            let distance_squared = to_centre.squared_norm();
                            if distance_squared < min_distance * min_distance {
                                let distance = distance_squared.sqrt();
                                min_distance = distance;
                                contact_normal = Some(normal * 0.125);
                            }
                        }

                        // Sphere against edges.
                        for (&v1, &v2) in vertices.iter()
                            .zip(vertices.iter().skip(1).chain(Some(&vertices[0]))) {
                            let edge = v2 - v1;
                            let on_edge = (position - v1).dot(&edge) / edge.dot(&edge);
                            if on_edge <= 0.0 || on_edge >= 1.0 {
                                continue;
                            }
                            let projection = v1 + edge * on_edge;
                            let edge_normal = position - projection;
                            let edge_distance_squared = edge_normal.squared_norm();

                            if edge_distance_squared < min_distance * min_distance {
                                min_distance = edge_distance_squared.sqrt();
                                contact_normal = Some(normal * 0.25);
                            }
                        }

                        // Apply force.
                        if let Some(contact_normal) = contact_normal {
                            let velocity_along_normal = velocity.dot(&contact_normal);
                            if velocity_along_normal < 0.0 {
                                *force -= contact_normal * (1.2 * velocity_along_normal) /
                                          (timestep * 0.5);
                            }
                            // *force += contact_normal * (radius - min_distance) * COMPRESS;
                        } else {
                        }
                    }

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
        let mut positions = &mut self.positions[..];
        let mut velocities = &mut self.velocities[..];

        assert!(positions.len() == velocities.len());
        let timestep = self.timestep;
        let update = |position: &mut Vec3f, velocity: &mut Vec3f| {
            *position += *velocity * timestep;
        };

        while positions.len() > 4 {
            update(&mut positions[0], &mut velocities[0]);
            update(&mut positions[1], &mut velocities[1]);
            update(&mut positions[2], &mut velocities[2]);
            update(&mut positions[3], &mut velocities[3]);

            positions = &mut deborrow(positions)[4..];
            velocities = &mut deborrow(velocities)[4..];
        }

        for i in 0..positions.len() {
            update(&mut positions[i], &mut velocities[i]);
        }

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

fn is_point_inside_triangle(verts: &[Vec3f; 3], point: &Vec3f) -> bool {
    let u = verts[1] - verts[0];
    let v = verts[2] - verts[0];
    let n = u.cross(v);
    let w = *point - verts[0];
    let n2 = n.squared_norm();

    let gamma = u.cross(w).dot(&n) / n2;
    let beta = w.cross(v).dot(&n) / n2;
    let alpha = 1.0 - gamma - beta;

    0.0 <= alpha && alpha <= 1.0 && 0.0 <= gamma && gamma <= 1.0 && 0.0 <= beta && beta <= 1.0
}
