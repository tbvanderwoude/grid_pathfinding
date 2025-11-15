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

#[cfg(test)]
mod tests {
    use grid_util::{Rect, ValueGrid};

    use crate::ALLOW_CORNER_CUTTING;

    use super::*;

    /// Asserts that the case in which start and goal are equal is handled correctly.
    #[test]
    fn equal_start_goal() {
        for allow_diag in [false, true] {
            let mut pathing_grid: PathingGrid = PathingGrid::new(1, 1, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.generate_components();
            let solver = AstarSolver::new();
            let start = Point::new(0, 0);
            let path = solver
                .get_path_single_goal(&mut pathing_grid, start, start, false)
                .unwrap();
            assert!(path.len() == 1);
        }
    }

    /// Asserts that the optimal 4 step solution is found.
    #[test]
    fn solve_simple_problem() {
        let arr = if ALLOW_CORNER_CUTTING {
            [(false, 5), (true, 4)]
        } else {
            [(false, 5), (true, 5)]
        };
        for (allow_diag, expected) in arr {
            let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.set(1, 1, true);
            pathing_grid.generate_components();
            let solver = AstarSolver::new();

            let start = Point::new(0, 0);
            let end = Point::new(2, 2);
            let path = solver
                .get_path_single_goal(&mut pathing_grid, start, end, false)
                .unwrap();
            assert!(path.len() == expected);
        }
    }

    #[test]
    fn test_multiple_goals() {
        let arr = if ALLOW_CORNER_CUTTING {
            [(false, 7), (true, 5)]
        } else {
            [(false, 7), (true, 6)]
        };
        for (allow_diag, expected) in arr {
            let mut pathing_grid: PathingGrid = PathingGrid::new(5, 5, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.set(1, 1, true);
            pathing_grid.generate_components();
            let solver = AstarSolver::new();
            let start = Point::new(0, 0);
            let goal_1 = Point::new(4, 4);
            let goal_2 = Point::new(3, 3);
            let goals = vec![&goal_1, &goal_2];
            let (selected_goal, path) = solver
                .get_path_multiple_goals(&mut pathing_grid, start, goals)
                .unwrap();
            assert_eq!(selected_goal, Point::new(3, 3));
            assert!(path.len() == expected);
        }
    }

    #[test]
    fn test_complex() {
        let arr = if ALLOW_CORNER_CUTTING {
            [(false, 15), (true, 10)]
        } else {
            [(false, 15), (true, 11)]
        };
        for (allow_diag, expected) in arr {
            let mut pathing_grid: PathingGrid = PathingGrid::new(10, 10, false);
            pathing_grid.set_rect(Rect::new(1, 1, 1, 1), true);
            pathing_grid.set_rect(Rect::new(5, 0, 1, 1), true);
            pathing_grid.set_rect(Rect::new(0, 5, 1, 1), true);
            pathing_grid.set_rect(Rect::new(8, 8, 1, 1), true);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.generate_components();
            let solver = AstarSolver::new();

            let start = Point::new(0, 0);
            let end = Point::new(7, 7);
            let path = solver
                .get_path_single_goal(&mut pathing_grid, start, end, false)
                .unwrap();
            assert!(path.len() == expected);
        }
    }

    // Tests whether allowing diagonals has the expected effect on path existence in a minimal setting.
    #[test]
    fn test_diagonal_switch_path() {
        //  ___
        // | #|
        // |# |
        //  __
        let mut pathing_grid: PathingGrid = PathingGrid::new(2, 2, true);
        pathing_grid.allow_diagonal_move = false;
        let mut pathing_grid_diag: PathingGrid = PathingGrid::new(2, 2, true);
        for pathing_grid in [&mut pathing_grid, &mut pathing_grid_diag] {
            pathing_grid.set(0, 0, false);
            pathing_grid.set(1, 1, false);
            pathing_grid.generate_components();
        }
        let solver = AstarSolver::new();

        let start = Point::new(0, 0);
        let goal = Point::new(1, 1);
        let path = solver.get_path_single_goal(&mut pathing_grid, start, goal, false);
        let path_diag = solver.get_path_single_goal(&mut pathing_grid_diag, start, goal, false);
        assert!(path.is_none());
        if ALLOW_CORNER_CUTTING {
            assert!(path_diag.is_some());
        } else {
            assert!(path_diag.is_none());
        }
    }
}
