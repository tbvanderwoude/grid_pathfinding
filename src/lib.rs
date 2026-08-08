//! # grid_pathfinding
//!
//! A grid-based pathfinding system. Implements
//! [Jump Point Search](https://en.wikipedia.org/wiki/Jump_point_search) with
//! [improved pruning rules](https://www.researchgate.net/publication/287338108_Improving_jump_point_search)
//! for speedy
//! pathfinding. Note that this assumes a uniform-cost grid. Pre-computes
//! [connected components](https://en.wikipedia.org/wiki/Component_(graph_theory))
//! to avoid flood-filling behaviour if no path exists.

mod astar_jps;
pub mod pathing_grid;
pub mod solver;
use astar_jps::SearchContext;
use core::fmt;
use grid_util::direction::Direction;
use grid_util::grid::ValueGrid;
use grid_util::point::Point;
use std::collections::VecDeque;

use crate::pathing_grid::PathingGrid;
use crate::solver::jps::JPSSolver;
use crate::solver::GridSolver;

pub const DEFAULT_CUT_CORNERS: bool = true;
pub const DEFAULT_IMPROVED_PRUNING: bool = false;
const EQUAL_EDGE_COST: bool = false;
const N_SMALLVEC_SIZE: usize = 8;

// Costs for diagonal and cardinal moves.
// Values for unequal costs approximating a ratio D/C of sqrt(2) are from
// https://github.com/riscy/a_star_on_grids
const D: i32 = if EQUAL_EDGE_COST { 1 } else { 99 };
const C: i32 = if EQUAL_EDGE_COST { 1 } else { 70 };
const E: i32 = 2 * C - D;

/// Converts the integer cost to an approximate floating point equivalent where cardinal directions have cost 1.0.
pub fn convert_cost_to_unit_cost_float(cost: i32) -> f64 {
    (cost as f64) / (C as f64)
}

/// Helper function for debugging binary representations of neighborhoods.
pub fn explain_bin_neighborhood(nn: u8) {
    for i in 0..8_i32 {
        let x = nn & (1 << i) != 0;
        let dir = Direction::try_from(i.rem_euclid(8)).unwrap();
        if x {
            println!("\t\t {dir:?}");
        }
    }
}

/// Turns waypoints into a path on the grid which can be followed step by step. Due to symmetry this
/// is typically one of many ways to follow the waypoints.
pub fn waypoints_to_path(waypoints: Vec<Point>) -> Vec<Point> {
    let mut waypoint_queue = waypoints.into_iter().collect::<VecDeque<Point>>();
    let mut path: Vec<Point> = Vec::new();
    let mut current = waypoint_queue.pop_front().unwrap();
    path.push(current);
    for next in waypoint_queue {
        while current.move_distance(&next) >= 1 {
            let delta = current.dir(&next);
            current = current + delta;
            path.push(current);
        }
    }
    path
}

/// [PathingGrid] maintains information about components using a [UnionFind] structure in addition to the raw
/// [bool] grid values in the [BoolGrid] that determine whether a space is occupied ([true]) or
/// empty ([false]). It also records neighbours in [u8] format for fast lookups during search.
/// Implements [Grid] by building on [BoolGrid].
#[derive(Clone, Debug)]
pub struct Pathfinder<const ALLOW_DIAGONAL: bool, const CUT_CORNERS: bool = DEFAULT_CUT_CORNERS> {
    pub solver: JPSSolver,
    pub grid: PathingGrid<ALLOW_DIAGONAL, CUT_CORNERS>,
}

impl<const ALLOW_DIAGONAL: bool, const CUT_CORNERS: bool> Pathfinder<ALLOW_DIAGONAL,CUT_CORNERS>{
    pub fn set_improved_pruning(&mut self, improved_pruning: bool){
        self.solver.improved_pruning = improved_pruning;
    }

}
impl<const ALLOW_DIAGONAL: bool, const CUT_CORNERS: bool> Default
    for Pathfinder<ALLOW_DIAGONAL, CUT_CORNERS>
{
    fn default() -> Pathfinder<ALLOW_DIAGONAL, CUT_CORNERS> {
        let mut grid = Pathfinder {
            solver: JPSSolver::default(),
            grid: PathingGrid::default(),
        };
        grid.initialize();
        grid
    }
}
impl<const ALLOW_DIAGONAL: bool, const CUT_CORNERS: bool> Pathfinder<ALLOW_DIAGONAL, CUT_CORNERS> {
    /// Computes a path from start to goal using JPS. If approximate is [true], then it will
    /// path to one of the neighbours of the goal, which is useful if the goal itself is
    /// blocked. If diagonals are allowed, the heuristic used computes the path cost
    /// of taking the maximal number of diagonal moves before continuing straight. If diagonals are not allowed, the [Manhattan distance](https://en.wikipedia.org/wiki/Taxicab_geometry)
    /// is used instead (see [heuristic](Self::heuristic)). This can be
    /// specified by setting [allow_diagonal_move](Self::allow_diagonal_move).
    /// The heuristic will be scaled by [heuristic_factor](Self::heuristic_factor) which can be used to trade optimality for faster solving for many practical problems, a technique
    /// called Weighted A*. In pathfinding language, a factor greater than
    /// 1.0 will make the heuristic [inadmissible](https://en.wikipedia.org/wiki/Admissible_heuristic), a requirement for solution optimality. By default,
    /// the [heuristic_factor](Self::heuristic_factor) is 1.0 which gives optimal solutions.
    pub fn get_path_single_goal(&mut self, start: Point, goal: Point) -> Option<Vec<Point>> {
        self.solver
            .get_path_single_goal(&mut self.grid, start, goal)
    }
    pub fn get_path_single_goal_approximate(
        &mut self,
        start: Point,
        goal: Point,
    ) -> Option<Vec<Point>> {
        self.solver
            .get_path_single_goal_approximate(&mut self.grid, start, goal)
    }

    /// Computes a path from the start to one of the given goals and returns the selected goal in addition to the found path. Otherwise behaves similar to [get_path_single_goal](Self::get_path_single_goal).
    pub fn get_path_multiple_goals(
        &mut self,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        self.solver
            .get_path_multiple_goals(&mut self.grid, start, goals)
    }
    /// The raw waypoints (jump points) from which [get_path_multiple_goals](Self::get_path_multiple_goals) makes a path.
    pub fn get_waypoints_multiple_goals(
        &mut self,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        self.solver
            .get_path_multiple_goals(&mut self.grid, start, goals)
    }
    /// The raw waypoints (jump points) from which [get_path_single_goal](Self::get_path_single_goal) makes a path.
    pub fn get_waypoints_single_goal(&mut self, start: Point, goal: Point) -> Option<Vec<Point>> {
        self.solver
            .get_waypoints_single_goal(&mut self.grid, start, goal)
    }
    /// The raw waypoints (jump points) from which [get_path_single_goal](Self::get_path_single_goal) makes a path.
    pub fn get_waypoints_single_goal_approximate(
        &mut self,
        start: Point,
        goal: Point,
    ) -> Option<Vec<Point>> {
        self.solver
            .get_waypoints_single_goal_approximate(&mut self.grid, start, goal)
    }
    /// Regenerates the components if they are marked as dirty.
    pub fn update(&mut self) {
        self.grid.update();
    }

    pub fn initialize(&mut self) {
        self.solver.initialize(&self.grid);
    }

    pub fn generate_components(&mut self) {
        self.grid.generate_components();
    }
}
impl<const ALLOW_DIAGONAL: bool, const CUT_CORNERS: bool> fmt::Display
    for Pathfinder<ALLOW_DIAGONAL, CUT_CORNERS>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.grid.fmt(f)
    }
}

impl<const ALLOW_DIAGONAL: bool, const CUT_CORNERS: bool> ValueGrid<bool>
    for Pathfinder<ALLOW_DIAGONAL, CUT_CORNERS>
{
    fn new(width: usize, height: usize, default_value: bool) -> Self {
        let grid = PathingGrid::new(width, height, default_value);
        let solver = JPSSolver::new(&grid, true);
        Pathfinder { grid, solver }
    }
    fn get(&self, x: i32, y: i32) -> bool {
        self.grid.get(x, y)
    }
    /// Updates a position on the grid. Joins newly connected components and flags the components
    /// as dirty if components are (potentially) broken apart into multiple.
    fn set(&mut self, x: i32, y: i32, blocked: bool) {
        self.grid.set(x, y, blocked);
        self.solver.set(x, y, blocked, &self.grid);
    }
    fn width(&self) -> usize {
        self.grid.width()
    }
    fn height(&self) -> usize {
        self.grid.height()
    }
}
