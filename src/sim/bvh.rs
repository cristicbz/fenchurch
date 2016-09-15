use idcontain::{Id, OptionId, IdVec};
use math::Vec3f;
use super::utils::{pmin_pmax, AssertOrd};
use num::Zero;
use std::f32;

#[derive(Copy, Clone, Debug)]
pub struct Aabb {
    min: Vec3f,
    max: Vec3f,
}

impl Aabb {
    pub fn negative() -> Self {
        Aabb {
            min: Vec3f::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Vec3f::new(-f32::INFINITY, -f32::INFINITY, -f32::INFINITY),
        }
    }

    pub fn of_sphere(position: &Vec3f, radius: f32) -> Self {
        assert!(radius > 0.0);
        let radius = Vec3f::new(radius, radius, radius);
        Aabb {
            min: *position - radius,
            max: *position + radius,
        }
    }

    pub fn union<'a, I: IntoIterator<Item = &'a Aabb>>(bbs: I) -> Self {
        let mut aabb = Aabb::negative();
        aabb.add_aabbs(bbs);
        aabb
    }

    pub fn new(point1: Vec3f, point2: Vec3f) -> Self {
        let tuples = [pmin_pmax(point1[0], point2[0]),
                      pmin_pmax(point1[1], point2[1]),
                      pmin_pmax(point1[2], point2[2])];
        Aabb {
            min: Vec3f::new(tuples[0].0, tuples[1].0, tuples[2].0),
            max: Vec3f::new(tuples[0].1, tuples[1].1, tuples[2].1),
        }
    }

    pub fn area(&self) -> f32 {
        let edges = self.max - self.min;
        2.0 * (edges[0] * edges[1] + edges[1] * edges[2] + edges[0] * edges[2])
    }

    #[inline]
    pub fn intersects_sphere(&self, position: Vec3f, radius: f32) -> bool {
        let min_diff = self.min - position;
        let max_diff = position - self.max;

        return min_diff[0] <= radius && min_diff[1] <= radius && min_diff[2] <= radius &&
               max_diff[0] <= radius && max_diff[1] <= radius &&
               max_diff[2] <= radius;
    }

    pub fn longest_axis(&self) -> usize {
        let diagonal = self.max - self.min;
        if diagonal[0] > diagonal[1] {
            if diagonal[0] > diagonal[2] { 0 } else { 2 }
        } else {
            if diagonal[1] > diagonal[2] { 1 } else { 2 }
        }
    }

    pub fn centroid(&self) -> Vec3f {
        (self.min + self.max) * 0.5
    }

    pub fn min(&self) -> &Vec3f {
        &self.min
    }

    pub fn max(&self) -> &Vec3f {
        &self.max
    }

    pub fn add_point(&mut self, point: Vec3f) {
        let Aabb { ref mut min, ref mut max } = *self;

        for i in 0..3 {
            if point[i] < min[i] {
                min[i] = point[i];
            } else if point[i] > max[i] {
                max[i] = point[i];
            }
        }
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

    pub fn add_aabbs<'a, I: IntoIterator<Item = &'a Aabb>>(&mut self, bbs: I) {
        for bb in bbs {
            self.add_aabb(bb);
        }
    }

    pub fn add_sphere(&mut self, position: &Vec3f, radius: f32) {
        self.add_aabb(&Self::of_sphere(position, radius));
    }
}


const INVALID_ID: u32 = 0xff_ff_ff_ff;
const MAX_CAPACITY: u32 = INVALID_ID;

#[derive(Debug)]
struct Node {
    aabb: Aabb,
    child: u32,
    leaf_end: u32,
}

impl Node {
    fn new(aabb: Aabb) -> Self {
        Node {
            aabb: aabb,
            child: INVALID_ID,
            leaf_end: INVALID_ID,
        }
    }
}

struct Work {
    index: usize,
    start: usize,
    end: usize,
}

pub struct Bvh {
    nodes: Vec<Node>,
    leaves: Vec<u32>,
    work_stack: Vec<Work>,
    bbs: Vec<Aabb>,
    centroids: Vec<Vec3f>,
}


fn median3(a: f32, b: f32, c: f32) -> f32 {
    if a < b {
        if a >= c {
            a
        } else if b < c {
            b
        } else {
            c
        }
    } else if a < c {
        a
    } else if b >= c {
        b
    } else {
        c
    }
}

impl Bvh {
    pub fn new() -> Self {
        Bvh {
            nodes: Vec::new(),
            leaves: Vec::new(),
            bbs: Vec::new(),
            work_stack: Vec::with_capacity(64),
            centroids: Vec::new(),
        }
    }

    pub fn rebuild<I: IntoIterator<Item = Aabb>>(&mut self, bbs_iter: I) {
        const MIN_LEAVES: usize = 10;

        let Bvh { ref mut nodes,
                  ref mut leaves,
                  ref mut bbs,
                  ref mut work_stack,
                  ref mut centroids,
                  .. } = *self;

        nodes.clear();
        centroids.clear();

        bbs.clear();
        bbs.extend(bbs_iter);
        let num_bbs = bbs.len();
        assert!(num_bbs <= (MAX_CAPACITY as usize));
        if num_bbs == 0 {
            return;
        }

        leaves.clear();
        leaves.extend(0..num_bbs as u32);
        centroids.extend(bbs.iter().map(|bb| bb.centroid()));

        nodes.reserve(bbs.len() / 2);
        nodes.push(Node::new(Aabb::union(&bbs[..])));
        work_stack.push(Work {
            index: 0,
            start: 0,
            end: num_bbs,
        });
        while let Some(Work { index, start, end }) = work_stack.pop() {
            assert!(start < end);
            assert!(end <= bbs.len());
            assert!(bbs.len() == leaves.len());
            assert!(bbs.len() == centroids.len());

            let aabb = nodes[index].aabb;

            let (child, leaf_end) = if end - start <= MIN_LEAVES {
                (start as u32, end as u32)
            } else {
                let longest_axis = aabb.longest_axis();
                let (limit, left_bb, right_bb) = binning_sah_limit(longest_axis,
                                                                   &aabb,
                                                                   &centroids[start..end],
                                                                   &bbs[start..end]);
                // let limit = median3_limit(longest_axis, &centroids[start..end]);


                let mut split = start;
                for i_leaf in start..end {
                    if centroids[i_leaf][longest_axis] <= limit {
                        bbs.swap(split, i_leaf);
                        leaves.swap(split, i_leaf);
                        centroids.swap(split, i_leaf);
                        split += 1;
                    }
                }


                if split == end || split == start {
                    (start as u32, end as u32)
                } else {
                    // let left_bb = Aabb::union(&bbs[start..split]);
                    // let right_bb = Aabb::union(&bbs[split..end]);
                    let index1 = nodes.len();
                    let index2 = nodes.len() + 1;
                    nodes.push(Node::new(left_bb));
                    nodes.push(Node::new(right_bb));
                    work_stack.push(Work {
                        index: index2,
                        start: split,
                        end: end,
                    });
                    work_stack.push(Work {
                        index: index1,
                        start: start,
                        end: split,
                    });

                    (index1 as u32, INVALID_ID)
                }
            };

            nodes[index].child = child;
            nodes[index].leaf_end = leaf_end;
        }
    }

    pub fn intersect_sphere<'a, 'b>(&'a self,
                                    iter_stack: &'b mut Vec<usize>,
                                    position: Vec3f,
                                    radius: f32)
                                    -> SphereIntersectIter<'a, 'b> {
        iter_stack.clear();
        if !self.nodes.is_empty() {
            iter_stack.push(0);
        }
        SphereIntersectIter {
            bvh: self,
            iter_stack: iter_stack,
            position: position,
            radius: radius,
            state: State::FindIntersecting,
        }
    }
}

pub struct SphereIntersectIter<'a, 'b> {
    bvh: &'a Bvh,
    iter_stack: &'b mut Vec<usize>,
    position: Vec3f,
    radius: f32,
    state: State,
}

impl<'a, 'b> SphereIntersectIter<'a, 'b> {
    fn first_intersection(&mut self) -> Option<usize> {
        while let Some(node_index) = self.iter_stack.pop() {
            if self.bvh.nodes[node_index].aabb.intersects_sphere(self.position, self.radius) {
                return Some(node_index);
            }
        }
        None
    }

    fn find_intersecting(&mut self) -> State {
        loop {
            let node_index = match self.first_intersection() {
                Some(index) => index,
                None => return State::Done,
            };
            let node = &self.bvh.nodes[node_index];
            if node.leaf_end == INVALID_ID {
                self.iter_stack.push(node.child as usize);
                self.iter_stack.push(node.child as usize + 1);
            } else {
                return State::YieldLeaves(node.child as usize, node.leaf_end as usize);
            }
        }
    }
}

enum State {
    FindIntersecting,
    YieldLeaves(usize, usize),
    Done,
}

impl<'a, 'b> Iterator for SphereIntersectIter<'a, 'b> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            match self.state {
                State::YieldLeaves(start, end) => {
                    if start < end {
                        self.state = State::YieldLeaves(start + 1, end);
                        return Some(self.bvh.leaves[start] as usize);
                    } else {
                        self.state = State::FindIntersecting;
                    }
                }
                State::FindIntersecting => self.state = self.find_intersecting(),
                State::Done => return None,
            }
        }
    }
}


fn median3_limit(axis: usize, centroids: &[Vec3f]) -> f32 {
    median3(centroids[0][axis],
            centroids[centroids.len() / 2][axis],
            centroids[centroids.len() - 1][axis])
}

fn binning_sah_limit(axis: usize,
                     aabb: &Aabb,
                     centroids: &[Vec3f],
                     bbs: &[Aabb])
                     -> (f32, Aabb, Aabb) {
    const NUM_BINS: usize = 8;

    assert!(axis < 3);
    let min_limit = aabb.min()[axis];
    let max_limit = aabb.max()[axis];

    let mut bin_bbs = [Aabb::negative(); NUM_BINS];
    let mut bin_counts = [0u32; NUM_BINS];
    let mut left_bbs = [Aabb::negative(); NUM_BINS];
    let mut left_costs = [0.0; NUM_BINS];

    let binning_const = NUM_BINS as f32 * (1.0 - 1e-5) / (max_limit - min_limit);
    for (index, &centroid) in centroids.iter().enumerate() {
        let bin_index = (binning_const * (centroid[axis] - min_limit)) as usize;
        bin_counts[bin_index] += 1;
        bin_bbs[bin_index].add_aabb(&bbs[index]);
    }

    let mut left_bb = Aabb::negative();
    let mut left_count = 0;
    for bin_index in 0..NUM_BINS - 1 {
        left_bb.add_aabb(&bin_bbs[bin_index]);
        left_count += bin_counts[bin_index];
        left_bbs[bin_index] = left_bb;
        left_costs[bin_index] = left_bb.area() * (left_count as f32);
    }

    let mut best_bin_cost = f32::INFINITY;
    let mut best_bin_index = NUM_BINS + 1;
    let mut best_right_bb = Aabb::negative();
    let mut right_bb = Aabb::negative();
    let mut right_count = 0;
    for bin_index in (0..NUM_BINS - 1).rev() {
        right_bb.add_aabb(&bin_bbs[bin_index + 1]);
        right_count += bin_counts[bin_index + 1];
        let cost = left_costs[bin_index] + right_bb.area() * (right_count as f32);

        if cost < best_bin_cost {
            best_bin_cost = cost;
            best_bin_index = bin_index;
            best_right_bb = right_bb;
        }
    }

    ((best_bin_index + 1) as f32 / binning_const + min_limit,
     left_bbs[best_bin_index],
     best_right_bb)
}

#[cfg(test)]
mod tests {
    use math::Vec3f;
    use super::Aabb;
    use num::Zero;
    use super::median3;

    #[test]
    fn test_aabb_union() {
        let aabb = Aabb::union(&[Aabb::of_sphere(&Vec3f::zero(), 1.0),
                                 Aabb::of_sphere(&Vec3f::zero(), 2.0),
                                 Aabb::of_sphere(&Vec3f::new(1.0, 0.0, 0.0), 1.0),
                                 Aabb::of_sphere(&Vec3f::new(-1.0, 0.0, 0.0), 2.0),
                                 Aabb::of_sphere(&Vec3f::new(1.0, 2.0, 0.0), 1.0)]);

        assert_eq!(aabb.min(), &Vec3f::new(-3.0, -2.0, -2.0));
        assert_eq!(aabb.max(), &Vec3f::new(2.0, 3.0, 2.0));
    }

    #[test]
    fn test_aabb_sphere() {
        let aabb = Aabb::new(Vec3f::new(-1.0, -1.0, -1.0), Vec3f::new(1.0, 1.0, 1.0));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 0.0, 0.0), 1.0));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 0.0, 0.0), 0.1));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 0.0, 0.0), 10.0));

        assert!(aabb.intersects_sphere(Vec3f::new(2.0, 0.0, 0.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 2.0, 0.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 0.0, 2.0), 1.5));

        assert!(aabb.intersects_sphere(Vec3f::new(2.0, 2.0, 0.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 2.0, 2.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(2.0, 0.0, 2.0), 1.5));

        assert!(!aabb.intersects_sphere(Vec3f::new(2.0, 0.0, 0.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(0.0, 2.0, 0.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(0.0, 0.0, 2.0), 0.9));

        assert!(!aabb.intersects_sphere(Vec3f::new(2.0, 2.0, 0.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(0.0, 2.0, 2.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(2.0, 0.0, 2.0), 0.9));

        assert!(aabb.intersects_sphere(Vec3f::new(-2.0, 0.0, 0.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, -2.0, 0.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, 0.0, -2.0), 1.5));

        assert!(aabb.intersects_sphere(Vec3f::new(-2.0, -2.0, 0.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(0.0, -2.0, -2.0), 1.5));
        assert!(aabb.intersects_sphere(Vec3f::new(-2.0, 0.0, -2.0), 1.5));

        assert!(!aabb.intersects_sphere(Vec3f::new(-2.0, 0.0, 0.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(0.0, -2.0, 0.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(0.0, 0.0, -2.0), 0.9));

        assert!(!aabb.intersects_sphere(Vec3f::new(-2.0, -2.0, 0.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(0.0, -2.0, -2.0), 0.9));
        assert!(!aabb.intersects_sphere(Vec3f::new(-2.0, 0.0, -2.0), 0.9));
    }

    #[test]
    fn test_median3() {
        assert_eq!(median3(0.0, 1.0, 2.0), 1.0);
        assert_eq!(median3(0.0, 2.0, 1.0), 1.0);
        assert_eq!(median3(1.0, 0.0, 2.0), 1.0);
        assert_eq!(median3(1.0, 2.0, 0.0), 1.0);
        assert_eq!(median3(2.0, 0.0, 1.0), 1.0);
        assert_eq!(median3(2.0, 1.0, 0.0), 1.0);

        assert_eq!(median3(2.0, 2.0, 0.0), 2.0);
        assert_eq!(median3(2.0, 0.0, 2.0), 2.0);
        assert_eq!(median3(0.0, 2.0, 2.0), 2.0);

        assert_eq!(median3(2.0, 2.0, 2.0), 2.0);
    }
}
