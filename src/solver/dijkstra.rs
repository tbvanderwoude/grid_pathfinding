use grid_util::Point;
use smallvec::SmallVec;

use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};

#[derive(Clone, Debug)]
pub struct DijkstraSolver;

impl GridSolver for DijkstraSolver {
    type Successors = SmallVec<[(Point, i32); N_SMALLVEC_SIZE]>;

    fn successors<F>(
        &self,
        grid: &PathingGrid,
        _parent: Option<&Point>,
        node: &Point,
        _goal: &F,
    ) -> Self::Successors
    where
        F: Fn(&Point) -> bool,
    {
        grid.neighborhood_points_and_cost(node)
    }

    /// Just the cost times a heuristic factor.
    fn heuristic(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32 {
        0
    }
}
