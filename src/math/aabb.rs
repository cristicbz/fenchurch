use super::Vec3f;
use std::f32;

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Aabb {
    min: Vec3f,
    max: Vec3f,
}

impl Aabb {
    #[inline]
    pub fn negative() -> Self {
        Aabb {
            min: Vec3f::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Vec3f::new(-f32::INFINITY, -f32::INFINITY, -f32::INFINITY),
        }
    }

    #[inline]
    pub fn of_sphere(position: &Vec3f, radius: f32) -> Self {
        assert!(radius > 0.0);
        let radius = Vec3f::new(radius, radius, radius);
        Aabb {
            min: *position - radius,
            max: *position + radius,
        }
    }

    pub fn of_points(points: &[Vec3f]) -> Self {
        let mut aabb = Aabb::negative();
        for point in points {
            aabb.min[0] = aabb.min[0].min(point[0]);
            aabb.min[1] = aabb.min[1].min(point[1]);
            aabb.min[2] = aabb.min[2].min(point[2]);

            aabb.max[0] = aabb.max[0].max(point[0]);
            aabb.max[1] = aabb.max[1].max(point[1]);
            aabb.max[2] = aabb.max[2].max(point[2]);
        }
        aabb
    }

    #[inline]
    pub fn union(bbs: &[Aabb]) -> Self {
        bbs.iter()
            .cloned()
            .fold(Aabb::negative(), Aabb::merge)
    }

    #[inline]
    pub fn area(&self) -> f32 {
        let edges = self.max - self.min;
        2.0 * (edges[0] * edges[1] + edges[1] * edges[2] + edges[0] * edges[2])
    }

    #[inline]
    pub fn intersects_sphere(&self, position: &Vec3f, radius: f32) -> bool {
        return self.min[0] - position[0] <= radius && self.min[1] - position[1] <= radius &&
               self.min[2] - position[2] <= radius &&
               position[0] - self.max[0] <= radius &&
               position[1] - self.max[1] <= radius &&
               position[2] - self.max[2] <= radius;
    }

    #[inline]
    pub fn longest_axis(&self) -> usize {
        let diagonal = self.max - self.min;
        if diagonal[0] > diagonal[1] {
            if diagonal[0] > diagonal[2] { 0 } else { 2 }
        } else {
            if diagonal[1] > diagonal[2] { 1 } else { 2 }
        }
    }

    #[inline]
    pub fn centroid(&self) -> Vec3f {
        (self.min + self.max) * 0.5
    }

    #[inline]
    pub fn min(&self) -> &Vec3f {
        &self.min
    }

    #[inline]
    pub fn max(&self) -> &Vec3f {
        &self.max
    }

    #[inline]
    pub fn merge(mut self, other: Aabb) -> Self {
        self.add_aabb(&other);
        self
    }

    #[inline]
    pub fn add_aabb(&mut self, other: &Aabb) {
        let Aabb { ref mut min, ref mut max } = *self;
        let Aabb { min: other_min, max: other_max } = *other;

        if other_min[0] < min[0] {
            min[0] = other_min[0];
        }

        if other_min[1] < min[1] {
            min[1] = other_min[1];
        }

        if other_min[2] < min[2] {
            min[2] = other_min[2];
        }

        if other_max[0] > max[0] {
            max[0] = other_max[0];
        }

        if other_max[1] > max[1] {
            max[1] = other_max[1];
        }

        if other_max[2] > max[2] {
            max[2] = other_max[2];
        }
    }
}
