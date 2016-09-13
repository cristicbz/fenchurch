use idcontain::{IdVec, Id};
use math::{Vec3f, Vector};
use num::Zero;

use std::cmp;

const GRAVITY: f32 = 1.0;
const COMPRESS: f32 = 5.0;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EntityId(Id<ComponentIndexes>);

pub struct Simulation {
    entities: IdVec<ComponentIndexes>,

    timestep: f32,
    colours: Vec<Vec3f>,
    forces: Vec<Vec3f>,
    masses: Vec<f32>,
    positions: Vec<Vec3f>,
    radii: Vec<f32>,
    reverse_lookup: Vec<EntityId>,
    velocities: Vec<Vec3f>,
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
    pub fn with_capacity(capacity: u32) -> Self {
        let capacity = capacity as usize;
        Simulation {
            entities: IdVec::with_capacity(capacity),

            timestep: 1. / 300.,

            colours: Vec::with_capacity(capacity),
            forces: Vec::with_capacity(capacity),
            masses: Vec::with_capacity(capacity),
            positions: Vec::with_capacity(capacity),
            radii: Vec::with_capacity(capacity),
            reverse_lookup: Vec::with_capacity(capacity),
            velocities: Vec::with_capacity(capacity),
        }
    }

    pub fn add(&mut self, entity: NewEntity) -> EntityId {
        let base_index = self.positions.len() as u32;
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
            let swap_index = self.positions.len() - 1;
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

    pub fn update(&mut self, _delta_time: f32) {
        self.start_step();
        self.forces.clear();
        self.forces.resize(self.positions.len(), Vec3f::zero());
        self.compute_forces();
        self.end_step();
        self.trim_and_recentre();
    }

    fn compute_forces(&mut self) {
        let positions = &self.positions[..];
        let velocities = &self.velocities[..];
        let masses = &self.masses[..];
        let radii = &self.radii[..];
        let mut forces = &mut self.forces[..];

        assert!(positions.len() == velocities.len());
        assert!(positions.len() == forces.len());
        assert!(positions.len() == masses.len());

        for i in 0..positions.len() {
            for j in (i + 1)..positions.len() {
                let diff = positions[j] - positions[i];
                let squared_norm = diff.squared_norm();
                let distance = squared_norm.sqrt();
                let magnitude = GRAVITY / (squared_norm * distance);

                let surface_distance = distance - radii[i] - radii[j];
                if surface_distance < 0.03 {
                    let diff = diff.normalized();
                    let relative = velocities[j] - velocities[i];
                    let dot = diff.dot(&relative);
                    let normal = diff * if dot < 0.0 { dot } else { 0.0 };
                    let tangent = relative - normal;
                    let tangent_speed = tangent.norm();

                    let force = normal * 0.9 / (2.0 * 0.5 * self.timestep);
                    forces[i] += force;
                    forces[j] -= force;

                    if tangent_speed > 1e-3 {
                        let tangent = tangent / tangent_speed;
                        let force = tangent * tangent_speed * 0.01;
                        forces[i] -= force;
                        forces[j] += force;
                    }

                    let force = diff * (COMPRESS * surface_distance);
                    forces[i] += force;
                    forces[j] -= force;
                } else {
                    forces[i] += diff * (magnitude * masses[j]);
                    forces[j] -= diff * (magnitude * masses[i]);
                }
            }
        }
    }


    fn start_step(&mut self) {
        let mut positions = &mut self.positions[..];
        let mut velocities = &mut self.velocities[..];
        let mut forces = &mut self.forces[..];
        let mut masses = &mut self.masses[..];

        assert!(positions.len() == velocities.len());
        assert!(positions.len() == forces.len());
        assert!(positions.len() == masses.len());

        let timestep = self.timestep;
        let half_timestep = timestep * 0.5;

        let update = |position: &mut Vec3f, velocity: &mut Vec3f, force: Vec3f, mass: f32| {
            *velocity += force * (half_timestep / mass);
            *position += *velocity * timestep;
        };

        while positions.len() > 4 {
            update(&mut positions[0], &mut velocities[0], forces[0], masses[0]);
            update(&mut positions[1], &mut velocities[1], forces[1], masses[1]);
            update(&mut positions[2], &mut velocities[2], forces[2], masses[2]);
            update(&mut positions[3], &mut velocities[3], forces[3], masses[3]);

            positions = &mut deborrow(positions)[4..];
            velocities = &mut deborrow(velocities)[4..];
            forces = &mut deborrow(forces)[4..];
            masses = &mut deborrow(masses)[4..];
        }

        for i in 0..positions.len() {
            update(&mut positions[i], &mut velocities[i], forces[i], masses[i]);
        }
    }

    fn end_step(&mut self) {
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

    fn trim_and_recentre(&mut self) {
        let mut removals = Vec::new();
        for (i_position, position) in self.positions.iter_mut().enumerate() {
            if position.squared_norm() > 300.0 * 300.0 {
                removals.push(self.reverse_lookup[i_position]);
            }
        }

        for removal in removals {
            self.remove(removal);
            info!("Remove entity {:?}", removal);
        }

        let position_mean: Vec3f = self.positions
            .iter()
            .fold((1.0, Vec3f::zero()),
                  |(n, mean), &v| (n + 1.0, mean + (v - mean) / n))
            .1;
        let velocity_mean: Vec3f = self.velocities
            .iter()
            .fold((1.0, Vec3f::zero()),
                  |(n, mean), &v| (n + 1.0, mean + (v - mean) / n))
            .1;

        for position in self.positions.iter_mut() {
            *position -= position_mean;
        }

        for velocity in self.positions.iter_mut() {
            *velocity -= velocity_mean;
        }

    }
}

fn deborrow<T>(value: T) -> T {
    value
}
