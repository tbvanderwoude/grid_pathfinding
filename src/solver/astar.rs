use grid_util::Point;
use smallvec::SmallVec;

use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};

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
    fn heuristic<const ALLOW_DIAGONAL: bool>(&self, p1: &Point, p2: &Point) -> i32 {
        (self.cost::<ALLOW_DIAGONAL>(p1, p2) as f32 * self.heuristic_factor) as i32
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
        let mut pathing_grid: PathingGrid<false> = PathingGrid::new(1, 1, false);
        pathing_grid.generate_components();
        let solver = AstarSolver::new();
        let start = Point::new(0, 0);
        let path = solver
            .get_path_single_goal(&mut pathing_grid, start, start)
            .unwrap();
        assert!(path.len() == 1);
    }

    /// Asserts that the case in which start and goal are equal is handled correctly.
    #[test]
    fn equal_start_goal_diagonal() {
        let mut pathing_grid: PathingGrid<true> = PathingGrid::new(1, 1, false);
        pathing_grid.generate_components();
        let solver = AstarSolver::new();
        let start = Point::new(0, 0);
        let path = solver
            .get_path_single_goal(&mut pathing_grid, start, start)
            .unwrap();
        assert!(path.len() == 1);
    }

    /// Asserts that the optimal 4 step solution is found.
    #[test]
    fn solve_simple_problem() {
        let expected = 5;
        let mut pathing_grid: PathingGrid<false> = PathingGrid::new(3, 3, false);
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let solver = AstarSolver::new();

        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        let path = solver
            .get_path_single_goal(&mut pathing_grid, start, end)
            .unwrap();
        assert!(path.len() == expected);
    }

    /// Asserts that the optimal 4 step solution is found.
    #[test]
    fn solve_simple_problem_diagonal() {
        let expected = if ALLOW_CORNER_CUTTING { 4 } else { 5 };
        let mut pathing_grid: PathingGrid<true> = PathingGrid::new(3, 3, false);
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let solver = AstarSolver::new();

        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        let path = solver
            .get_path_single_goal(&mut pathing_grid, start, end)
            .unwrap();
        assert!(path.len() == expected);
    }

    #[test]
    fn test_multiple_goals() {
        let expected = 7;
        let mut pathing_grid: PathingGrid<false> = PathingGrid::new(5, 5, false);
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

    #[test]
    fn test_multiple_goal_diagonal() {
        let expected = if ALLOW_CORNER_CUTTING { 5 } else { 6 };
        let mut pathing_grid: PathingGrid<true> = PathingGrid::new(5, 5, false);
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
    #[test]
    fn test_complex() {
        let expected = 15;
        let mut pathing_grid: PathingGrid<false> = PathingGrid::new(10, 10, false);
        pathing_grid.set_rect(Rect::new(1, 1, 1, 1), true);
        pathing_grid.set_rect(Rect::new(5, 0, 1, 1), true);
        pathing_grid.set_rect(Rect::new(0, 5, 1, 1), true);
        pathing_grid.set_rect(Rect::new(8, 8, 1, 1), true);
        pathing_grid.generate_components();
        let solver = AstarSolver::new();

        let start = Point::new(0, 0);
        let end = Point::new(7, 7);
        let path = solver
            .get_path_single_goal(&mut pathing_grid, start, end)
            .unwrap();
        assert!(path.len() == expected);
    }
    #[test]
    fn test_complex_diagonal() {
        let expected = if ALLOW_CORNER_CUTTING { 10 } else { 11 };
        let mut pathing_grid: PathingGrid<true> = PathingGrid::new(10, 10, false);
        pathing_grid.set_rect(Rect::new(1, 1, 1, 1), true);
        pathing_grid.set_rect(Rect::new(5, 0, 1, 1), true);
        pathing_grid.set_rect(Rect::new(0, 5, 1, 1), true);
        pathing_grid.set_rect(Rect::new(8, 8, 1, 1), true);
        pathing_grid.generate_components();
        let solver = AstarSolver::new();

        let start = Point::new(0, 0);
        let end = Point::new(7, 7);
        let path = solver
            .get_path_single_goal(&mut pathing_grid, start, end)
            .unwrap();
        assert!(path.len() == expected);
    }
    // Tests whether allowing diagonals has the expected effect on path existence in a minimal setting.
    #[test]
    fn test_diagonal_switch_path() {
        //  ___
        // | #|
        // |# |
        //  __
        let mut pathing_grid: PathingGrid<false> = PathingGrid::new(2, 2, true);
        let mut pathing_grid_diag: PathingGrid<true> = PathingGrid::new(2, 2, true);
        pathing_grid.set(0, 0, false);
        pathing_grid.set(1, 1, false);
        pathing_grid.generate_components();
        pathing_grid_diag.set(0, 0, false);
        pathing_grid_diag.set(1, 1, false);
        pathing_grid_diag.generate_components();
        let solver = AstarSolver::new();

        let start = Point::new(0, 0);
        let goal = Point::new(1, 1);
        let path = solver.get_path_single_goal(&mut pathing_grid, start, goal);
        let path_diag = solver.get_path_single_goal(&mut pathing_grid_diag, start, goal);
        assert!(path.is_none());
        if ALLOW_CORNER_CUTTING {
            assert!(path_diag.is_some());
        } else {
            assert!(path_diag.is_none());
        }
    }
}
