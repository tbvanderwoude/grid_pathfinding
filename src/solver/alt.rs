use std::i32;

use crate::solver::dijkstra::DijkstraSolver;
use grid_util::{Point, SimpleValueGrid, ValueGrid};
use smallvec::SmallVec;

use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};

#[derive(Clone, Debug)]
pub struct ALTSolver {
    landmarks: Vec<Point>,
    landmark_distances: Vec<SimpleValueGrid<i32>>,
}
impl ALTSolver {
    pub fn new<const ALLOW_DIAGONAL: bool>(
        landmarks: Vec<Point>,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> ALTSolver {
        let d = DijkstraSolver;
        let mut landmark_distances = Vec::new();
        for landmark in &landmarks {
            let mut dists = SimpleValueGrid::new(grid.width(), grid.height(), i32::MAX);
            let mut ct = grid.context.lock().unwrap();
            ct.astar_jps(
                &landmark,
                |parent, node| d.successors(grid, *parent, node, &|_| false),
                |_| 0,
                |_| false,
            )
            .map(|(v, _c)| v);
            for (p, (_, c)) in &ct.parents {
                dists.set_point(*p, *c);
            }
            landmark_distances.push(dists);
        }
        ALTSolver {
            landmarks,
            landmark_distances,
        }
    }
}

impl GridSolver for ALTSolver {
    type Successors = SmallVec<[(Point, i32); N_SMALLVEC_SIZE]>;

    fn successors<const ALLOW_DIAGONAL: bool, F>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        _parent: Option<&Point>,
        node: &Point,
        _goal: &F,
    ) -> Self::Successors
    where
        F: Fn(&Point) -> bool,
    {
        grid.neighborhood_points_and_cost(node)
    }

    /// Just the normal cost times a heuristic factor.
    fn heuristic<const ALLOW_DIAGONAL: bool>(
        &self,
        _: &PathingGrid<ALLOW_DIAGONAL>,
        p1: &Point,
        p2: &Point,
    ) -> i32 {
        self.landmark_distances
            .iter()
            .map(|x| (x.get_point(*p1) - x.get_point(*p2)).abs())
            .max()
            .unwrap()
    }
}
