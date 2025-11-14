use super::pathing_grid::PathingGrid;
use super::*;
use core::fmt;
use grid_util::grid::{BoolGrid, SimpleValueGrid, ValueGrid};
use grid_util::point::Point;
use petgraph::unionfind::UnionFind;
use smallvec::SmallVec;
use std::sync::{Arc, Mutex};

pub trait GridSolver {
    /// Container type for successors; you can keep this fixed to SmallVec if you prefer.
    type Successors: IntoIterator<Item = (Point, i32)>;

    fn heuristic(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32;
    fn successors(
        &self,
        grid: &PathingGrid,
        _parent: Option<&Point>,
        node: &Point,
    ) -> Self::Successors;

    fn get_path_single_goal(
        &self,
        grid: &mut PathingGrid,
        start: Point,
        goal: Point,
        approximate: bool,
    ) -> Option<Vec<Point>> {
        self.get_waypoints_single_goal(grid, start, goal, approximate)
            .map(waypoints_to_path)
    }
    /// The raw waypoints (jump points) from which [get_path_single_goal](Self::get_path_single_goal) makes a path.
    fn get_waypoints_single_goal(
        &self,
        grid: &mut PathingGrid,
        start: Point,
        goal: Point,
        approximate: bool,
    ) -> Option<Vec<Point>> {
        if approximate {
            // Check if start and one of the goal neighbours are on the same connected component.
            if grid.neighbours_unreachable(&start, &goal) {
                // No neigbhours of the goal are reachable from the start
                return None;
            }
            // A neighbour of the goal can be reached, compute a path
            let mut ct = grid.context.lock().unwrap();
            ct.astar_jps(
                &start,
                |parent, node| self.successors(grid, *parent, node),
                |point| self.heuristic(grid, point, &goal),
                |point| self.heuristic(grid, point, &goal) <= if EQUAL_EDGE_COST { 1 } else { 99 },
            )
        } else {
            // Check if start and goal are on the same connected component.
            if grid.unreachable(&start, &goal) {
                return None;
            }
            // The goal is reachable from the start, compute a path
            let mut ct = grid.context.lock().unwrap();
            ct.astar_jps(
                &start,
                |parent, node| self.successors(grid, *parent, node),
                |point| self.heuristic(grid, point, &goal),
                |point| *point == goal,
            )
        }
        .map(|(v, _c)| v)
    }
    /// Computes a path from the start to one of the given goals and returns the selected goal in addition to the found path. Otherwise behaves similar to [get_path_single_goal](Self::get_path_single_goal).
    fn get_path_multiple_goals(
        &self,
        grid: &mut PathingGrid,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        self.get_waypoints_multiple_goals(grid, start, goals)
            .map(|(x, y)| (x, waypoints_to_path(y)))
    }
    /// The raw waypoints (jump points) from which [get_path_multiple_goals](Self::get_path_multiple_goals) makes a path.
    fn get_waypoints_multiple_goals(
        &self,
        grid: &mut PathingGrid,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        if goals.is_empty() {
            return None;
        }
        let mut ct = grid.context.lock().unwrap();
        let result = ct.astar_jps(
            &start,
            |parent, node| self.successors(grid, *parent, node),
            |point| {
                goals
                    .iter()
                    .map(|x| self.heuristic(grid, point, x))
                    .min()
                    .unwrap()
            },
            |point| goals.contains(&point),
        );
        result.map(|(v, _c)| (*v.last().unwrap(), v))
    }
}

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

    fn successors(
        &self,
        grid: &PathingGrid,
        _parent: Option<&Point>,
        node: &Point,
    ) -> Self::Successors {
        grid.neighborhood_points_and_cost(node)
    }
    /// Uses C as cost for cardinal (straight) moves and D for diagonal moves.
    fn heuristic(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32 {
        ((if grid.allow_diagonal_move {
            let delta_x = (p1.x - p2.x).abs();
            let delta_y = (p1.y - p2.y).abs();
            // Formula from https://github.com/riscy/a_star_on_grids
            // to efficiently compute the cost of a path taking the maximal amount
            // of diagonal steps before going straight
            (E * (delta_x - delta_y).abs() + D * (delta_x + delta_y)) / 2
        } else {
            p1.manhattan_distance(p2) * C
        }) as f32
            * self.heuristic_factor) as i32
    }
}
