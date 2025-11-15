use grid_util::Point;
use smallvec::SmallVec;

use crate::{pathing_grid::PathingGrid, solver::GridSolver, C, D, E, N_SMALLVEC_SIZE};

#[derive(Clone, Debug)]
pub struct AstarSolver {
    pub heuristic_factor: f32,
}

impl AstarSolver {
    pub fn new() -> AstarSolver {
        AstarSolver {
            heuristic_factor: 1.0,
        }
    }
}

impl GridSolver for AstarSolver {
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

    /// Uses C as cost for cardinal (straight) moves and D for diagonal moves.
    fn cost(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32 {
        if grid.allow_diagonal_move {
            let delta_x = (p1.x - p2.x).abs();
            let delta_y = (p1.y - p2.y).abs();
            // Formula from https://github.com/riscy/a_star_on_grids
            // to efficiently compute the cost of a path taking the maximal amount
            // of diagonal steps before going straight
            (E * (delta_x - delta_y).abs() + D * (delta_x + delta_y)) / 2
        } else {
            p1.manhattan_distance(p2) * C
        }
    }

    /// Just the cost times a heuristic factor.
    fn heuristic(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32 {
        (self.cost(grid, p1, p2) as f32 * self.heuristic_factor) as i32
    }
}
