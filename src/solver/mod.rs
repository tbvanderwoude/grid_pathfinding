use crate::{C, EQUAL_EDGE_COST, pathing_grid::PathingGrid, waypoints_to_path};
use grid_util::Point;

pub mod astar;
pub mod jps;

/// Converts the integer cost to an approximate floating point equivalent where cardinal directions have cost 1.0.
pub fn convert_cost_to_unit_cost_float(cost: i32) -> f64 {
    (cost as f64) / (C as f64)
}

pub trait GridSolver {
    type Successors: IntoIterator<Item = (Point, i32)>;

    fn heuristic(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32;

    fn cost(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32;

    fn successors<F>(
        &self,
        grid: &PathingGrid,
        parent: Option<&Point>,
        node: &Point,
        goal: &F,
    ) -> Self::Successors
    where
        F: Fn(&Point) -> bool;

    fn get_path_cost(&self, path: Vec<Point>, pathing_grid: &PathingGrid) -> i32 {
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
    fn get_path_cost_float(&self, path: Vec<Point>, pathing_grid: &PathingGrid) -> f64 {
        convert_cost_to_unit_cost_float(self.get_path_cost(path, pathing_grid))
    }
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
                |parent, node| {
                    self.successors(grid, *parent, node, &|node_pos| {
                        self.heuristic(grid, node_pos, &goal)
                            <= if EQUAL_EDGE_COST { 1 } else { 99 }
                    })
                },
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
                |parent, node| self.successors(grid, *parent, node, &|node_pos| *node_pos == goal),
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
