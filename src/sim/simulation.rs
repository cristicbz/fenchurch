use idcontain::{IdVec, Id};
use math::{Vec3f, Vector, Vec4f, Aabb};
use num::Zero;
use std::f32::consts;
use super::bvh::Bvh;
use super::frame_timers::{FrameTimers, FrameTimerId};
use rayon::prelude::*;
use std::cell::RefCell;

const DRAG_COEFFICIENT: f32 = 1.0;
const COMPRESS: f32 = 10000.0;
const EXPLODE: f32 = 5000.0;
const GRAVITY: f32 = 20.0;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EntityId(Id<ComponentIndexes>);

pub struct Simulation {
    entities: IdVec<ComponentIndexes>,

    bvh_timer: FrameTimerId,
    forces_timer: FrameTimerId,

    bvh: Bvh,

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
    pub fn with_capacity(frame_timers: &mut FrameTimers, capacity: u32) -> Self {
        let capacity = capacity as usize;
        Simulation {
            entities: IdVec::with_capacity(capacity),
            bvh: Bvh::new(),

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
        self.bvh
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
        let positions = &self.positions[..];
        let velocities = &self.velocities[..];
        let radii = &self.radii[..];
        let bvh = &self.bvh;
        let explode = self.explode;
        let timestep = self.timestep;

        let planes = [Vec4f::new(1.0, 0.0, 0.0, 5.0),
                      Vec4f::new(-1.0, 0.0, 0.0, 5.0),
                      Vec4f::new(0.0, 0.0, 1.0, 5.0),
                      Vec4f::new(0.0, 0.0, -1.0, 5.0),
                      Vec4f::new(0.0, 1.0, 0.0, 0.0)];

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

                for plane in &planes {
                    let plane_distance = plane.xyz().dot(&position) + plane[3];
                    let distance_difference = plane_distance - radius;
                    if distance_difference < 0.0 {
                        let normal_dot = velocity.dot(&plane.xyz());
                        if normal_dot < 0.0 {
                            let normal_dot = normal_dot.max(-0.5 / (timestep * 0.5));
                            *force += velocity * normal_dot * 1.0;
                        }
                        *force += plane.xyz() * (-distance_difference * COMPRESS);
                    }
                }

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
                    for j_body in bvh.intersect_sphere(&mut intersection_stack, position, radius) {
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
