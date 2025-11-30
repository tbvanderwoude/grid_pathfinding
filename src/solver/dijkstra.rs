use grid_util::Point;
use smallvec::SmallVec;

use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};

#[derive(Clone, Debug)]
pub struct DijkstraSolver;

impl GridSolver for DijkstraSolver {
    type Successors = SmallVec<[(Point, i32); N_SMALLVEC_SIZE]>;

    fn successors<const ALLOW_DIAGONAL: bool, F>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        _: Option<&Point>,
        node: &Point,
        _: &F,
    ) -> Self::Successors
    where
        F: Fn(&Point) -> bool,
    {
        grid.neighborhood_points_and_cost(node)
    }

    /// Just the cost times a heuristic factor.
    fn heuristic<const ALLOW_DIAGONAL: bool>(&self, _: &Point, _: &Point) -> i32 {
        0
    }
}
