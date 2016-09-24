use math::{Aabb, Vec3f};
use rayon;
use std::f32;
use std::marker::PhantomData;
use std::mem;
use super::atomic_mut_indexer::AtomicMutIndexer;

pub enum Two {}
pub enum Four {}
pub enum Six {}
pub enum Eight {}
pub enum Sixteen {}

pub trait SpecifyNumber {
    #[inline]
    fn specify_number() -> usize;
}
impl SpecifyNumber for Two {
    #[inline]
    fn specify_number() -> usize {
        2
    }
}
impl SpecifyNumber for Four {
    #[inline]
    fn specify_number() -> usize {
        4
    }
}
impl SpecifyNumber for Six {
    #[inline]
    fn specify_number() -> usize {
        6
    }
}
impl SpecifyNumber for Eight {
    #[inline]
    fn specify_number() -> usize {
        8
    }
}
impl SpecifyNumber for Sixteen {
    #[inline]
    fn specify_number() -> usize {
        16
    }
}

pub trait Array<T: Copy + Sync + Send>: AsRef<[T]> + AsMut<[T]> + Sync + Send {
    fn of(value: T) -> Self;
}
impl<T: Copy + Sync + Send> Array<T> for [T; 2] {
    #[inline]
    fn of(value: T) -> Self {
        [value; 2]
    }
}
impl<T: Copy + Sync + Send> Array<T> for [T; 4] {
    #[inline]
    fn of(value: T) -> Self {
        [value; 4]
    }
}
impl<T: Copy + Sync + Send> Array<T> for [T; 6] {
    #[inline]
    fn of(value: T) -> Self {
        [value; 6]
    }
}
impl<T: Copy + Sync + Send> Array<T> for [T; 8] {
    #[inline]
    fn of(value: T) -> Self {
        [value; 8]
    }
}
impl<T: Copy + Sync + Send> Array<T> for [T; 16] {
    #[inline]
    fn of(value: T) -> Self {
        [value; 16]
    }
}


pub trait SpecifyArraySize<T: Copy + Sync + Send> {
    type ArrayOfSize: Array<T>;
}

impl<T: Copy + Sync + Send> SpecifyArraySize<T> for Two {
    type ArrayOfSize = [T; 2];
}
impl<T: Copy + Sync + Send> SpecifyArraySize<T> for Four {
    type ArrayOfSize = [T; 4];
}
impl<T: Copy + Sync + Send> SpecifyArraySize<T> for Six {
    type ArrayOfSize = [T; 6];
}
impl<T: Copy + Sync + Send> SpecifyArraySize<T> for Eight {
    type ArrayOfSize = [T; 8];
}
impl<T: Copy + Sync + Send> SpecifyArraySize<T> for Sixteen {
    type ArrayOfSize = [T; 16];
}

pub struct MinLeaves<N: SpecifyNumber>(PhantomData<N>);
pub trait SpecifyMinLeaves {
    fn min_leaves() -> usize;
}
impl<N: SpecifyNumber> SpecifyMinLeaves for MinLeaves<N> {
    #[inline]
    fn min_leaves() -> usize {
        N::specify_number()
    }
}

pub trait PartitionHeuristic {
    fn partition(aabb: &Aabb,
                 bbs: &mut [Aabb],
                 centroids: &mut [Vec3f],
                 leaves: &mut [u32])
                 -> Option<(usize, Aabb, Aabb)>;
}

pub trait SahBinLimits {
    fn sah_bin_limits(aabb: &Aabb,
                      bbs: &[Aabb],
                      centroids: &[Vec3f],
                      leaves: &[u32])
                      -> (usize, f32, f32);
}

pub enum CentroidAabbLimit {}
impl SahBinLimits for CentroidAabbLimit {
    #[inline]
    fn sah_bin_limits(_aabb: &Aabb,
                      _bbs: &[Aabb],
                      centroids: &[Vec3f],
                      _leaves: &[u32])
                      -> (usize, f32, f32) {
        let centroid_bb = Aabb::of_points(centroids);
        let axis = centroid_bb.longest_axis();
        (axis, centroid_bb.min()[axis], centroid_bb.max()[axis])
    }
}
pub enum TotalAabbLimit {}
impl SahBinLimits for TotalAabbLimit {
    #[inline]
    fn sah_bin_limits(aabb: &Aabb,
                      _bbs: &[Aabb],
                      _centroids: &[Vec3f],
                      _leaves: &[u32])
                      -> (usize, f32, f32) {
        let axis = aabb.longest_axis();
        (axis, aabb.min()[axis], aabb.max()[axis])
    }
}

pub trait SpecifyBinCount
    : SpecifyNumber + SpecifyArraySize<f32> + SpecifyArraySize<u32> + SpecifyArraySize<Aabb>
    {
    type CostArray: Array<f32>;
    type CountArray: Array<u32>;
    type AabbArray: Array<Aabb>;
}
impl<N> SpecifyBinCount for N
    where N: SpecifyNumber + SpecifyArraySize<f32> + SpecifyArraySize<u32> + SpecifyArraySize<Aabb>
{
    type CostArray = <N as SpecifyArraySize<f32>>::ArrayOfSize;
    type CountArray = <N as SpecifyArraySize<u32>>::ArrayOfSize;
    type AabbArray = <N as SpecifyArraySize<Aabb>>::ArrayOfSize;
}

pub struct BinnedSahPartition<N: SpecifyBinCount, Limits: SahBinLimits> {
    _phantom: PhantomData<(N, Limits)>,
}

pub trait SpecifyOptions {
    type MinLeaves: SpecifyMinLeaves;
    type Heuristic: PartitionHeuristic;
}

pub struct Options<MinLeaves: SpecifyMinLeaves, Heuristic: PartitionHeuristic> {
    _phantom: PhantomData<(MinLeaves, Heuristic)>,
}

impl<MinLeaves: SpecifyMinLeaves, Heuristic: PartitionHeuristic> SpecifyOptions for Options<MinLeaves, Heuristic> {
    type MinLeaves = MinLeaves;
    type Heuristic = Heuristic;
}

pub struct Bvh<O: SpecifyOptions> {
    nodes: Vec<Node>,
    leaves: Vec<u32>,
    centroids: Vec<Vec3f>,
    bbs: Vec<Aabb>,
    new_bbs: Vec<Aabb>,
    _phantom: PhantomData<O>,
}

impl<O: SpecifyOptions> Bvh<O> {
    pub fn new() -> Self {
        Bvh {
            nodes: Vec::new(),
            leaves: Vec::new(),
            bbs: Vec::new(),
            new_bbs: Vec::new(),
            centroids: Vec::new(),
            _phantom: PhantomData,
        }
    }

    pub fn rebuild<I: IntoIterator<Item = Aabb>>(&mut self, bbs_iter: I) {
        let Bvh { ref mut nodes,
                  ref mut leaves,
                  ref mut bbs,
                  ref mut new_bbs,
                  ref mut centroids,
                  .. } = *self;
        new_bbs.clear();
        new_bbs.extend(bbs_iter);
        let num_bbs = new_bbs.len();
        assert!(num_bbs <= (MAX_CAPACITY as usize));
        if num_bbs == 0 {
            bbs.clear();
            new_bbs.clear();
            nodes.clear();
            leaves.clear();
            centroids.clear();
            return;
        }

        if num_bbs != leaves.len() {
            mem::swap(bbs, new_bbs);
            leaves.clear();
            leaves.extend(0..num_bbs as u32);
        } else {
            bbs.clear();
            bbs.extend(leaves.iter().map(|&index| new_bbs[index as usize]));
        }
        centroids.clear();
        centroids.extend(bbs.iter().map(|bb| bb.centroid()));

        nodes.clear();
        if num_bbs <= O::MinLeaves::min_leaves() {
            nodes.push(Node {
                aabb: Aabb::union(&bbs[..]),
                child: 0,
                max_index: num_bbs as u32 - 1,
                leaf_end: num_bbs as u32,
            });
            return;
        }
        nodes.resize((num_bbs + 1) * 2, Node::new());
        let num_nodes = {
            let node_indexer = AtomicMutIndexer::new(nodes);
            let (root_index, root) = node_indexer.get().unwrap();
            assert_eq!(root_index, 0);
            let root_aabb = Aabb::union(&bbs[..]);
            root.aabb = root_aabb;
            root.max_index = num_bbs as u32 - 1;
            parallel_expand::<O>(&node_indexer,
                                 NodeExpansion {
                                     node: root,
                                     bbs: bbs,
                                     centroids: centroids,
                                     leaves: leaves,
                                     offset: 0,
                                 });
            node_indexer.done()
        };
        nodes.truncate(num_nodes);
    }

    #[inline]
    pub fn on_sphere_intersection<F>(&self,
                                     position: &Vec3f,
                                     radius: f32,
                                     min_index: usize,
                                     mut handler: F)
        where F: FnMut(usize)
    {
        if self.nodes.is_empty() || min_index >= self.leaves.len() ||
           !self.nodes[0].aabb.intersects_sphere(position, radius) {
            return;
        }
        self.sphere_intersector(&mut handler,
                                &position,
                                radius,
                                min_index as u32,
                                &self.nodes[0]);
    }

    fn sphere_intersector<'a, F>(&'a self,
                                 handler: &mut F,
                                 position: &Vec3f,
                                 radius: f32,
                                 min_index: u32,
                                 mut node: &'a Node)
        where F: FnMut(usize)
    {
        loop {
            if node.leaf_end != INVALID_ID {
                let (start, end) = (node.child as usize, node.leaf_end as usize);
                assert!(end <= self.leaves.len());
                assert!(end > start);
                for &leaf in &self.leaves[start..end] {
                    if leaf >= min_index {
                        handler(leaf as usize);
                    }
                }
                return;
            } else {
                let child_index = node.child as usize;
                let child2 = &self.nodes[child_index + 1];
                let child1 = &self.nodes[child_index];
                let intersect1 = child1.max_index >= min_index &&
                                 child1.aabb.intersects_sphere(position, radius);
                let intersect2 = child2.max_index >= min_index &&
                                 child2.aabb.intersects_sphere(position, radius);

                if intersect2 {
                    if intersect1 {
                        self.sphere_intersector(handler, position, radius, min_index, child1);
                    }
                    node = child2;
                } else if intersect1 {
                    node = child1;
                } else {
                    return;
                }
            }
        }
    }
}


const INVALID_ID: u32 = 0xff_ff_ff_ff;
const MAX_CAPACITY: u32 = INVALID_ID;

#[derive(Clone, Debug)]
struct Node {
    aabb: Aabb,
    max_index: u32,
    child: u32,
    leaf_end: u32,
}

impl Node {
    fn new() -> Self {
        Node {
            aabb: Aabb::negative(),
            child: INVALID_ID,
            leaf_end: INVALID_ID,
            max_index: INVALID_ID,
        }
    }
}

const SEQUENTIAL_EXPANSION_THRESHOLD: usize = 512;

struct NodeExpansion<'a> {
    node: &'a mut Node,
    bbs: &'a mut [Aabb],
    centroids: &'a mut [Vec3f],
    leaves: &'a mut [u32],
    offset: u32,
}

#[inline]
fn parallel_expand<'a, O: SpecifyOptions>(node_indexer: &'a AtomicMutIndexer<Node>,
                                          mut expansion: NodeExpansion<'a>) {
    while expansion.bbs.len() > SEQUENTIAL_EXPANSION_THRESHOLD {
        match expand_node::<O>(node_indexer, expansion) {
            (Some(e1), Some(e2)) => {
                rayon::join(|| parallel_expand::<O>(node_indexer, e1),
                            || parallel_expand::<O>(node_indexer, e2));
                return;
            }
            (Some(e), None) | (None, Some(e)) => expansion = e,
            (None, None) => return,
        }
    }
    sequential_expand::<O>(node_indexer, expansion);
}

fn sequential_expand<'a, O: SpecifyOptions>(node_indexer: &'a AtomicMutIndexer<Node>,
                                            expansion: NodeExpansion<'a>) {
    match expand_node::<O>(node_indexer, expansion) {
        (Some(e1), Some(e2)) => {
            sequential_expand::<O>(node_indexer, e1);
            sequential_expand::<O>(node_indexer, e2);
        }
        (Some(e), None) | (None, Some(e)) => return sequential_expand::<O>(node_indexer, e),
        (None, None) => return,
    }
}

fn expand_node<'a, O: SpecifyOptions>(node_indexer: &'a AtomicMutIndexer<Node>,
                                      expansion: NodeExpansion<'a>)
                                      -> (Option<NodeExpansion<'a>>, Option<NodeExpansion<'a>>) {
    let NodeExpansion { node, bbs, centroids, leaves, offset } = expansion;

    let min_leaves = O::MinLeaves::min_leaves();
    let len = bbs.len();
    assert!(len > min_leaves);
    assert_eq!(leaves.len(), len);
    assert_eq!(centroids.len(), len);

    let (split, left_bb, right_bb) =
        match O::Heuristic::partition(&node.aabb, bbs, centroids, leaves) {
            Some(partition) => partition,
            None => {
                node.child = offset;
                node.leaf_end = offset + len as u32;
                return (None, None);
            }
        };

    assert!(split > 0 && split < len);
    let (left_bbs, right_bbs) = bbs.split_at_mut(split);
    let (left_centroids, right_centroids) = centroids.split_at_mut(split);
    let (left_leaves, right_leaves) = leaves.split_at_mut(split);
    let (len, split) = (len as u32, split as u32);
    let (index1, child1, child2) = node_indexer.get2().expect("not enough preallocated nodes");

    node.child = index1 as u32;
    node.leaf_end = INVALID_ID;

    child1.aabb = left_bb;
    child1.max_index = *left_leaves.iter().max().expect("left_leaves shouldn't be empty");
    let left = if left_bbs.len() <= min_leaves {
        child1.child = offset;
        child1.leaf_end = offset + split;
        None
    } else {
        Some(NodeExpansion {
            node: child1,
            bbs: left_bbs,
            centroids: left_centroids,
            leaves: left_leaves,
            offset: offset,
        })
    };

    child2.aabb = right_bb;
    child2.max_index = *right_leaves.iter().max().expect("right_leaves shouldn't be empty");
    let right = if right_bbs.len() <= min_leaves {
        child2.child = offset + split;
        child2.leaf_end = offset + len;
        None
    } else {
        Some(NodeExpansion {
            node: child2,
            bbs: right_bbs,
            centroids: right_centroids,
            leaves: right_leaves,
            offset: offset + split,
        })
    };

    (left, right)
}

#[derive(Copy, Clone)]
struct Bins<N: SpecifyBinCount> {
    bbs: N::AabbArray,
    counts: N::CountArray,
}

impl<N: SpecifyBinCount> Bins<N> {
    fn identity() -> Self {
        Bins {
            bbs: N::AabbArray::of(Aabb::negative()),
            counts: N::CountArray::of(0),
        }
    }

    fn merge(mut self, other: Self) -> Self {
        for ((count, bb), (other_count, other_bb)) in self.counts
            .as_mut()
            .iter_mut()
            .zip(self.bbs.as_mut())
            .zip(other.counts.as_ref().iter().zip(other.bbs.as_ref())) {

            *count += *other_count;
            bb.add_aabb(other_bb);
        }
        self
    }

    fn create(binning_const: f32,
              min_limit: f32,
              axis: usize,
              bbs: &[Aabb],
              centroids: &[Vec3f])
              -> Self {
        let mut bins = Bins::<N>::identity();
        {
            let bin_counts = bins.counts.as_mut();
            let bin_bbs = bins.bbs.as_mut();
            for (bb, centroid) in bbs.iter().zip(centroids) {
                let bin_index = (binning_const * (centroid[axis] - min_limit)) as usize;
                bin_counts[bin_index] += 1;
                bin_bbs[bin_index].add_aabb(bb);
            }
        }
        bins
    }

    fn par_create(binning_const: f32,
                  min_limit: f32,
                  axis: usize,
                  bbs: &[Aabb],
                  centroids: &[Vec3f])
                  -> Self {
        const CHUNK_SIZE: usize = 4096;

        let len = bbs.len();
        assert!(centroids.len() == len);
        if len < CHUNK_SIZE {
            Self::create(binning_const, min_limit, axis, bbs, centroids)
        } else {
            let (left_bbs, right_bbs) = bbs.split_at(len / 2);
            let (left_centroids, right_centroids) = centroids.split_at(len / 2);
            let (left_bins, right_bins) = rayon::join(|| {
                Self::create(binning_const, min_limit, axis, left_bbs, left_centroids)
            },
                                                      || {
                Self::create(binning_const, min_limit, axis, right_bbs, right_centroids)
            });
            Self::merge(left_bins, right_bins)
        }
    }
}

impl<N: SpecifyBinCount, Limits: SahBinLimits> PartitionHeuristic for BinnedSahPartition<N,
                                                                                         Limits> {
    fn partition(aabb: &Aabb,
                 bbs: &mut [Aabb],
                 centroids: &mut [Vec3f],
                 leaves: &mut [u32])
                 -> Option<(usize, Aabb, Aabb)> {
        let len = bbs.len();
        assert!(len >= 2);
        assert_eq!(centroids.len(), len);
        assert_eq!(leaves.len(), len);

        let (axis, min_limit, max_limit) = Limits::sah_bin_limits(aabb, bbs, centroids, leaves);
        if max_limit - min_limit <= 1e-5 {
            return None;
        }

        let binning_const = (1.0 - 1e-5) * N::specify_number() as f32 / (max_limit - min_limit);
        let bins = Bins::<N>::par_create(binning_const, min_limit, axis, bbs, centroids);
        let bin_counts = bins.counts.as_ref();
        let bin_bbs = bins.bbs.as_ref();

        let num_bins = N::specify_number();
        let mut a_left_bbs = N::AabbArray::of(Aabb::negative());
        let mut a_left_costs = N::CostArray::of(0.0);

        {
            let left_bbs = a_left_bbs.as_mut();
            let left_costs = a_left_costs.as_mut();
            let mut left_bb = Aabb::negative();
            let mut left_count = 0u32;
            for bin_index in 0..num_bins - 1 {
                left_bb.add_aabb(&bin_bbs[bin_index]);
                left_count += bin_counts[bin_index];

                left_bbs[bin_index] = left_bb;
                left_costs[bin_index] = left_bb.area() * (left_count as f32);
            }
        }

        let left_bbs = a_left_bbs.as_ref();
        let left_costs = a_left_costs.as_ref();

        let mut best_bin_cost = f32::INFINITY;
        let mut best_bin_index = N::specify_number() + 1;
        let mut best_right_bb = Aabb::negative();
        {
            let mut right_bb = Aabb::negative();
            let mut right_count = 0u32;
            for bin_index in (0..num_bins - 1).rev() {
                right_bb.add_aabb(&bin_bbs[bin_index + 1]);
                right_count += bin_counts[bin_index + 1];
                let cost = left_costs[bin_index] + right_bb.area() * (right_count as f32);

                if cost < best_bin_cost {
                    best_bin_cost = cost;
                    best_bin_index = bin_index;
                    best_right_bb = right_bb;
                }
            }
        }

        if best_bin_cost >= len as f32 * aabb.area() {
            return None;
        }

        let len = bbs.len();
        assert!(axis < 3);
        assert!(len >= 2);
        assert_eq!(centroids.len(), len);
        assert_eq!(leaves.len(), len);
        let limit = (best_bin_index + 1) as f32;
        let mut split = 0;
        for i_leaf in 0..len {
            if binning_const * (centroids[i_leaf][axis] - min_limit) < limit {
                bbs.swap(split, i_leaf);
                centroids.swap(split, i_leaf);
                leaves.swap(split, i_leaf);
                split += 1;
            }
        }
        Some((split, left_bbs[best_bin_index], best_right_bb))
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
