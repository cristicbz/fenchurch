use math::{Aabb, Vec3f};
use rayon;
use rayon::prelude::*;
use std::f32;
use super::atomic_mut_indexer::AtomicMutIndexer;
use std::slice::Iter as SliceIter;

pub struct Bvh {
    nodes: Vec<Node>,
    leaves: Vec<u32>,
    bbs: Vec<Aabb>,
}

impl Bvh {
    pub fn new() -> Self {
        Bvh {
            nodes: Vec::new(),
            leaves: Vec::new(),
            bbs: Vec::new(),
        }
    }

    pub fn rebuild<I: IntoIterator<Item = Aabb>>(&mut self, bbs_iter: I) {
        let Bvh { ref mut nodes, ref mut leaves, ref mut bbs, .. } = *self;
        nodes.clear();
        leaves.clear();
        bbs.clear();

        bbs.extend(bbs_iter);
        let num_bbs = bbs.len();
        assert!(num_bbs <= (MAX_CAPACITY as usize));
        if num_bbs == 0 {
            return;
        }
        leaves.extend(0..num_bbs as u32);

        if num_bbs < MIN_LEAVES {
            nodes.push(Node {
                aabb: Aabb::union(&bbs[..]),
                child: 0,
                leaf_end: num_bbs as u32,
            });
            return;
        }

        nodes.resize((num_bbs + 1) * 2, Node::new());
        let num_nodes = {
            let node_indexer = AtomicMutIndexer::new(nodes);
            let (root_index, root) = node_indexer.get().unwrap();
            assert_eq!(root_index, 0);
            expand_node(root, &node_indexer, Aabb::union(&bbs[..]), bbs, leaves, 0);
            node_indexer.done()
        };
        nodes.truncate(num_nodes);
    }

    pub fn intersect_sphere<'a, 'b>(&'a self,
                                    iter_stack: &'b mut Vec<usize>,
                                    position: Vec3f,
                                    radius: f32)
                                    -> SphereIntersectIter<'a, 'b> {
        iter_stack.clear();
        if self.nodes.is_empty() || !self.nodes[0].aabb.intersects_sphere(position, radius) {
            SphereIntersectIter {
                bvh: self,
                iter_stack: iter_stack,
                position: position,
                radius: radius,
                state: State::Done,
            }
        } else {
            iter_stack.push(0);
            SphereIntersectIter {
                bvh: self,
                iter_stack: iter_stack,
                position: position,
                radius: radius,
                state: State::FindIntersecting,
            }
        }
    }
}

const INVALID_ID: u32 = 0xff_ff_ff_ff;
const MAX_CAPACITY: u32 = INVALID_ID;
const MIN_LEAVES: usize = 6;

#[derive(Clone, Debug)]
struct Node {
    aabb: Aabb,
    child: u32,
    leaf_end: u32,
}

impl Node {
    fn new() -> Self {
        Node {
            aabb: Aabb::negative(),
            child: INVALID_ID,
            leaf_end: INVALID_ID,
        }
    }
}

fn expand_node(node: &mut Node,
               node_indexer: &AtomicMutIndexer<Node>,
               aabb: Aabb,
               bbs: &mut [Aabb],
               leaves: &mut [u32],
               offset: u32) {
    let len = bbs.len();
    assert!(len >= MIN_LEAVES);
    assert!(leaves.len() == len);
    let longest_axis = aabb.longest_axis();
    // let limit = median3_limit(longest_axis, bbs);
    let (limit, left_bb, right_bb) = match binned_sah_limit(longest_axis, &aabb, bbs) {
        Some(split) => split,
        None => {
            *node = Node {
                aabb: aabb,
                child: offset,
                leaf_end: offset + len as u32,
            };
            return;
        }
    };

    let mut split = 0;
    for i_leaf in 0..len {
        if bbs[i_leaf].centroid()[longest_axis] < limit {
            bbs.swap(split, i_leaf);
            leaves.swap(split, i_leaf);
            split += 1;
        }
    }

    if split == 0 || split == len {
        split = len / 2;
    }
    let (left_bbs, right_bbs) = bbs.split_at_mut(split);
    let (left_leaves, right_leaves) = leaves.split_at_mut(split);

    // let (left_bb, right_bb) = (Aabb::union(&left_bbs), Aabb::union(&right_bbs));
    let (index1, child1, child2) = node_indexer.get2().unwrap();

    *node = Node {
        aabb: aabb,
        child: index1 as u32,
        leaf_end: INVALID_ID,
    };

    if left_bbs.len() < MIN_LEAVES && right_bbs.len() < MIN_LEAVES {
        *child1 = Node {
            aabb: left_bb,
            child: offset,
            leaf_end: offset + split as u32,
        };
        *child2 = Node {
            aabb: right_bb,
            child: offset + split as u32,
            leaf_end: offset + len as u32,
        };
    } else if left_bbs.len() < MIN_LEAVES {
        *child1 = Node {
            aabb: left_bb,
            child: offset,
            leaf_end: offset + split as u32,
        };
        expand_node(child2,
                    node_indexer,
                    right_bb,
                    right_bbs,
                    right_leaves,
                    offset + split as u32)
    } else if right_bbs.len() < MIN_LEAVES {
        *child2 = Node {
            aabb: right_bb,
            child: offset + split as u32,
            leaf_end: offset + len as u32,
        };
        expand_node(child1, node_indexer, left_bb, left_bbs, left_leaves, offset);
    } else {
        rayon::join(|| expand_node(child1, node_indexer, left_bb, left_bbs, left_leaves, offset),
                    || {
            expand_node(child2,
                        node_indexer,
                        right_bb,
                        right_bbs,
                        right_leaves,
                        offset + split as u32)
        });
    }
}

pub struct SphereIntersectIter<'a, 'b> {
    bvh: &'a Bvh,
    iter_stack: &'b mut Vec<usize>,
    position: Vec3f,
    radius: f32,
    state: State<'a>,
}

impl<'a, 'b> SphereIntersectIter<'a, 'b> {
    fn find_intersecting(&mut self) -> State<'a> {
        let SphereIntersectIter { position, radius, .. } = *self;
        while let Some(node_index) = self.iter_stack.pop() {
            let node = &self.bvh.nodes[node_index];
            if node.leaf_end != INVALID_ID {
                return State::YieldLeaves(
                    self.bvh.leaves[node.child as usize..node.leaf_end as usize].iter());
            } else {
                let child = node.child as usize;
                if self.bvh.nodes[child].aabb.intersects_sphere(position, radius) {
                    self.iter_stack.push(child);
                }
                if self.bvh.nodes[child + 1].aabb.intersects_sphere(position, radius) {
                    self.iter_stack.push(child + 1);
                }
            }
        }

        State::Done
    }
}

enum State<'a> {
    FindIntersecting,
    YieldLeaves(SliceIter<'a, u32>),
    Done,
}

impl<'a, 'b> Iterator for SphereIntersectIter<'a, 'b> {
    type Item = usize;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let next_state;
            match self.state {
                State::YieldLeaves(ref mut iter) => {
                    match iter.next() {
                        Some(&index) => return Some(index as usize),
                        None => next_state = State::FindIntersecting,
                    }
                }
                State::FindIntersecting => next_state = self.find_intersecting(),
                State::Done => return None,
            }
            self.state = next_state;
        }
    }
}

// fn median3_limit(axis: usize, bbs: &[Aabb]) -> f32 {
//    median3(bbs[0].centroid()[axis],
//            bbs[bbs.len() / 2].centroid()[axis],
//            bbs[bbs.len() - 1].centroid()[axis])
// }
// fn median3(a: f32, b: f32, c: f32) -> f32 {
//    if a < b {
//        if a >= c {
//            a
//        } else if b < c {
//            b
//        } else {
//            c
//        }
//    } else if a < c {
//        a
//    } else if b >= c {
//        b
//    } else {
//        c
//    }
// }

const NUM_BINS: usize = 8;

#[derive(Copy, Clone)]
struct Bins {
    bbs: [Aabb; NUM_BINS],
    counts: [u32; NUM_BINS],
}

impl Bins {
    fn identity() -> Self {
        Bins {
            bbs: [Aabb::negative(); NUM_BINS],
            counts: [0; NUM_BINS],
        }
    }

    fn merge(mut self, other: Bins) -> Self {
        for bin_index in 0..NUM_BINS {
            self.counts[bin_index] += other.counts[bin_index];
            self.bbs[bin_index].add_aabb(&other.bbs[bin_index]);
        }
        self
    }

    fn create(binning_const: f32, min_limit: f32, axis: usize, bbs: &[Aabb]) -> Self {
        let mut bins = Bins::identity();
        for bb in bbs {
            let centroid = bb.centroid()[axis];
            let bin_index = (binning_const * (centroid - min_limit)) as usize;
            bins.counts[bin_index] += 1;
            bins.bbs[bin_index].add_aabb(bb);
        }
        bins
    }
}


fn binned_sah_limit(axis: usize, aabb: &Aabb, bbs: &[Aabb]) -> Option<(f32, Aabb, Aabb)> {
    const CHUNK_SIZE: usize = 16384;
    assert!(axis < 3);
    let len = bbs.len();
    let min_limit = aabb.min()[axis];
    let max_limit = aabb.max()[axis];

    if max_limit - min_limit <= 1e-3 {
        return None;
    }

    let binning_const = NUM_BINS as f32 * (1.0 - 1e-3) / (max_limit - min_limit);
    let bins = if len < CHUNK_SIZE * 2 {
        bbs.par_chunks(CHUNK_SIZE)
            .weight_max()
            .map(|bbs| Bins::create(binning_const, min_limit, axis, bbs))
            .reduce_with_identity(Bins::identity(), Bins::merge)
    } else {
        Bins::create(binning_const, min_limit, axis, bbs)
    };

    let mut left_bbs = [Aabb::negative(); NUM_BINS - 1];
    let mut left_costs = [0.0; NUM_BINS - 1];
    let mut left_bb = Aabb::negative();
    let mut left_count = 0;
    for bin_index in 0..NUM_BINS - 1 {
        left_bb.add_aabb(&bins.bbs[bin_index]);
        left_count += bins.counts[bin_index];
        left_bbs[bin_index] = left_bb;
        left_costs[bin_index] = left_bb.area() * (left_count as f32);
    }

    let mut best_bin_cost = f32::INFINITY;
    let mut best_bin_index = NUM_BINS + 1;
    let mut best_right_bb = Aabb::negative();
    let mut right_bb = Aabb::negative();
    let mut right_count = 0;
    for bin_index in (0..NUM_BINS - 1).rev() {
        right_bb.add_aabb(&bins.bbs[bin_index + 1]);
        right_count += bins.counts[bin_index + 1];
        let cost = left_costs[bin_index] + right_bb.area() * (right_count as f32);

        if cost < best_bin_cost {
            best_bin_cost = cost;
            best_bin_index = bin_index;
            best_right_bb = right_bb;
        }
    }

    if best_bin_cost >= len as f32 * aabb.area() {
        None
    } else {
        Some(((best_bin_index as f32 + 1.0) / binning_const + min_limit,
              left_bbs[best_bin_index],
              best_right_bb))
    }
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
