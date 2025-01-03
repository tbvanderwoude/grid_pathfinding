use fxhash::FxBuildHasher;
/// This module implements a variant of
/// [pathfinding's astar function](https://docs.rs/pathfinding/latest/pathfinding/directed/astar/index.html)
/// which enables the JPS implementation to generate successors based on the parent if there is one
/// as it should.
use indexmap::map::Entry::{Occupied, Vacant};
use indexmap::IndexMap;
use num_traits::Zero;

type FxIndexMap<K, V> = IndexMap<K, V, FxBuildHasher>;

use log::warn;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

use std::hash::Hash;

#[derive(Clone, Debug)]

struct SearchNode<K> {
    estimated_cost: K,
    cost: K,
    index: usize,
}

impl<K: PartialEq> Eq for SearchNode<K> {}

impl<K: PartialEq> PartialEq for SearchNode<K> {
    fn eq(&self, other: &Self) -> bool {
        self.estimated_cost.eq(&other.estimated_cost) && self.cost.eq(&other.cost)
    }
}

impl<K: Ord> PartialOrd for SearchNode<K> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<K: Ord> Ord for SearchNode<K> {
    fn cmp(&self, other: &Self) -> Ordering {
        // First orders per estimated cost, then creates subordering
        // based on cost, favoring exploration of smallest cost nodes first
        match other.estimated_cost.cmp(&self.estimated_cost) {
            Ordering::Equal => self.cost.cmp(&other.cost),
            // Uncommenting this gives the opposite tie-breaking effect
            // Ordering::Equal => other.cost.cmp(&self.cost),
            s => s,
        }
    }
}

fn reverse_path<N, V, F>(parents: &FxIndexMap<N, V>, mut parent: F, start: usize) -> Vec<N>
where
    N: Eq + Hash + Clone,
    F: FnMut(&V) -> usize,
{
    let mut path: Vec<N> = itertools::unfold(start, |i| {
        parents.get_index(*i).map(|(node, value)| {
            *i = parent(value);
            node.clone()
        })
    })
    .collect();
    path.reverse();
    path
}

/// [AstarContext] represents the search fringe and node parent map, facilitating reuse of memory allocations.
#[derive(Clone, Debug)]
pub struct AstarContext<N, C> {
    fringe: BinaryHeap<SearchNode<C>>,
    parents: FxIndexMap<N, (usize, C)>,
}

impl<N, C> AstarContext<N, C>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
{
    pub fn new() -> AstarContext<N, C> {
        AstarContext {
            fringe: BinaryHeap::new(),
            parents: FxIndexMap::default(),
        }
    }
    pub fn astar_jps<FN, IN, FH, FS>(
        &mut self,
        start: &N,
        mut successors: FN,
        mut heuristic: FH,
        mut success: FS,
    ) -> Option<(Vec<N>, C)>
    where
        FN: FnMut(&Option<&N>, &N) -> IN,
        IN: IntoIterator<Item = (N, C)>,
        FH: FnMut(&N) -> C,
        FS: FnMut(&N) -> bool,
    {
        self.fringe.clear();
        self.parents.clear();
        self.fringe.push(SearchNode {
            estimated_cost: Zero::zero(),
            cost: Zero::zero(),
            index: 0,
        });
        self.parents
            .insert(start.clone(), (usize::MAX, Zero::zero()));
        while let Some(SearchNode { cost, index, .. }) = self.fringe.pop() {
            let successors = {
                let (node, &(parent_index, c)) = self.parents.get_index(index).unwrap();
                if success(node) {
                    let path = reverse_path(&self.parents, |&(p, _)| p, index);
                    return Some((path, cost));
                }
                // We may have inserted a node several time into the binary heap if we found
                // a better way to access it. Ensure that we are currently dealing with the
                // best path and discard the others.
                if cost > c {
                    continue;
                }
                let optional_parent_node = self.parents.get_index(parent_index).map(|x| x.0);

                successors(&optional_parent_node, node)
            };
            for (successor, move_cost) in successors {
                let new_cost = cost + move_cost;
                let h; // heuristic(&successor)
                let n; // index for successor
                match self.parents.entry(successor) {
                    Vacant(e) => {
                        h = heuristic(e.key());
                        n = e.index();
                        e.insert((index, new_cost));
                    }
                    Occupied(mut e) => {
                        if e.get().1 > new_cost {
                            h = heuristic(e.key());
                            n = e.index();
                            e.insert((index, new_cost));
                        } else {
                            continue;
                        }
                    }
                }

                self.fringe.push(SearchNode {
                    estimated_cost: new_cost + h,
                    cost: new_cost,
                    index: n,
                });
            }
        }
        warn!("Reachable goal could not be pathed to, is reachable graph correct?");
        None
    }
}

/// Standalone astar_jps function that creates an [AstarContext] object for backward-compatibility.
pub fn astar_jps<N, C, FN, IN, FH, FS>(
    start: &N,
    successors: FN,
    heuristic: FH,
    success: FS,
) -> Option<(Vec<N>, C)>
where
    N: Eq + Hash + Clone,
    C: Zero + Ord + Copy,
    FN: FnMut(&Option<&N>, &N) -> IN,
    IN: IntoIterator<Item = (N, C)>,
    FH: FnMut(&N) -> C,
    FS: FnMut(&N) -> bool,
{
    let mut search = AstarContext::new();
    search.astar_jps(start, successors, heuristic, success)
}
