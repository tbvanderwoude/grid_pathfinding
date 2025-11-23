use crate::{pathing_grid::PathingGrid, waypoints_to_path, C, D, E, EQUAL_EDGE_COST};
use grid_util::Point;

pub mod astar;
pub mod dijkstra;
pub mod jps;

/// Converts the integer cost to an approximate floating point equivalent where cardinal directions have cost 1.0.
pub fn convert_cost_to_unit_cost_float(cost: i32) -> f64 {
    (cost as f64) / (C as f64)
}

pub trait GridSolver {
    type Successors: IntoIterator<Item = (Point, i32)>;

    fn heuristic<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        p1: &Point,
        p2: &Point,
    ) -> i32;

    /// Uses C as cost for cardinal (straight) moves and D for diagonal moves.
    fn cost<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        p1: &Point,
        p2: &Point,
    ) -> i32 {
        if ALLOW_DIAGONAL {
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

    fn successors<const ALLOW_DIAGONAL: bool, F>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        parent: Option<&Point>,
        node: &Point,
        goal: &F,
    ) -> Self::Successors
    where
        F: Fn(&Point) -> bool;

    fn get_path_cost<const ALLOW_DIAGONAL: bool>(
        &self,
        path: &Vec<Point>,
        pathing_grid: &PathingGrid<ALLOW_DIAGONAL>,
    ) -> i32 {
        let mut v = path[0];
        let n = path.len();
        let mut total_cost_int = 0;
        for i in 1..n {
            let v_old = v;
            v = path[i];
            let cost = self.cost(&pathing_grid, &v_old, &v);
            total_cost_int += cost;
        }
        total_cost_int
    }
    fn get_path_cost_float<const ALLOW_DIAGONAL: bool>(
        &self,
        path: &Vec<Point>,
        pathing_grid: &PathingGrid<ALLOW_DIAGONAL>,
    ) -> f64 {
        convert_cost_to_unit_cost_float(self.get_path_cost(path, pathing_grid))
    }
    fn get_path_single_goal<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
        start: Point,
        goal: Point,
    ) -> Option<Vec<Point>> {
        self.get_waypoints_single_goal(grid, start, goal)
            .map(waypoints_to_path)
    }
    fn get_path_single_goal_approximate<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
        start: Point,
        goal: Point,
    ) -> Option<Vec<Point>> {
        self.get_waypoints_single_goal_approximate(grid, start, goal)
            .map(waypoints_to_path)
    }
    /// The raw waypoints (jump points) from which [get_path_single_goal](Self::get_path_single_goal) makes a path.
    fn get_waypoints_single_goal<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
        start: Point,
        goal: Point,
    ) -> Option<Vec<Point>> {
        // Check if start and goal are on the same connected component.
        if grid.unreachable(&start, &goal) {
            return None;
        }
        // The goal is reachable from the start, compute a path
        let mut ct = grid.context.lock().unwrap();
        ct.astar_jps(
            &start,
            |parent, node| self.successors(grid, *parent, node, &|node_pos| *node_pos == goal),
            |point| self.heuristic(grid, point, &goal),
            |point| *point == goal,
        )
        .map(|(v, _c)| v)
    }

    /// The raw waypoints (jump points) from which [get_path_single_goal](Self::get_path_single_goal) makes a path.
    fn get_waypoints_single_goal_approximate<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
        start: Point,
        goal: Point,
    ) -> Option<Vec<Point>> {
        // Check if start and one of the goal neighbours are on the same connected component.
        if grid.neighbours_unreachable(&start, &goal) {
            // No neigbhours of the goal are reachable from the start
            return None;
        }
        // A neighbour of the goal can be reached, compute a path
        let mut ct = grid.context.lock().unwrap();
        ct.astar_jps(
            &start,
            |parent, node| {
                self.successors(grid, *parent, node, &|node_pos| {
                    self.heuristic(grid, node_pos, &goal) <= if EQUAL_EDGE_COST { 1 } else { 99 }
                })
            },
            |point| self.heuristic(grid, point, &goal),
            |point| self.heuristic(grid, point, &goal) <= if EQUAL_EDGE_COST { 1 } else { 99 },
        )
        .map(|(v, _c)| v)
    }
    /// Computes a path from the start to one of the given goals and returns the selected goal in addition to the found path. Otherwise behaves similar to [get_path_single_goal](Self::get_path_single_goal).
    fn get_path_multiple_goals<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        self.get_waypoints_multiple_goals(grid, start, goals)
            .map(|(x, y)| (x, waypoints_to_path(y)))
    }
    /// The raw waypoints (jump points) from which [get_path_multiple_goals](Self::get_path_multiple_goals) makes a path.
    fn get_waypoints_multiple_goals<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        if goals.is_empty() {
            return None;
        }
        let mut ct = grid.context.lock().unwrap();
        let result = ct.astar_jps(
            &start,
            |parent, node| self.successors(grid, *parent, node, &|point| goals.contains(&point)),
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
