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

use grid_util::direction::Direction;
use grid_util::grid::{BoolGrid, Grid, SimpleGrid};
use grid_util::point::Point;
use petgraph::unionfind::UnionFind;

use crate::astar_jps::astar_jps;
use core::fmt;
use std::collections::VecDeque;

/// Turns waypoints into a path on the grid which can be followed step by step. Due to symmetry this
/// is typically one of many ways to follow the waypoints.
pub fn waypoints_to_path(waypoints: Vec<Point>) -> Vec<Point> {
    if DEBUG_PRINT{
        println!("Waypoints:\n\t{:?}",&waypoints)
    }
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
    if DEBUG_PRINT{
        println!("Path:\n\t{:?}",&path)
    }
    path
}

const DEBUG_PRINT: bool = false;

/// [PathingGrid] maintains information about components using a [UnionFind] structure in addition to the raw
/// [bool] grid values in the [BoolGrid] that determine whether a space is occupied ([true]) or
/// empty ([false]). It also records neighbours in [u8] format for fast lookups during search.
/// Implements [Grid] by building on [BoolGrid].
#[derive(Clone, Debug)]
pub struct PathingGrid {
    pub grid: BoolGrid,
    pub neighbours: SimpleGrid<u8>,
    pub components: UnionFind<usize>,
    pub components_dirty: bool,
    pub heuristic_factor: f32,
    pub improved_pruning: bool,
    pub allow_diagonal_move: bool,
}

impl Default for PathingGrid {
    fn default() -> PathingGrid {
        PathingGrid {
            grid: BoolGrid::default(),
            neighbours: SimpleGrid::default(),
            components: UnionFind::new(0),
            components_dirty: false,
            improved_pruning: true,
            heuristic_factor: 1.0,
            allow_diagonal_move: true,
        }
    }
}
impl PathingGrid {
    fn get_neighbours(&self, point: Point) -> Vec<Point> {
        if self.allow_diagonal_move {
            point.moore_neighborhood()
        } else {
            point.neumann_neighborhood()
        }
        .into_iter()
        .filter(|p| self.can_move_to(*p))
        .collect::<Vec<Point>>()
    }
    /// Use the move distance or manhattan distance depending on whether diagonal moves are allowed.
    pub fn heuristic(&self, p1: &Point, p2: &Point) -> i32 {
        if self.allow_diagonal_move {
            p1.move_distance(p2)
        } else {
            p1.manhattan_distance(p2)
        }
    }
    fn can_move_to(&self, pos: Point) -> bool {
        self.in_bounds(pos.x, pos.y) && !self.grid.get(pos.x as usize, pos.y as usize)
    }
    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && self.grid.index_in_bounds(x as usize, y as usize)
    }
    /// The neighbour indexing used here corresponds to that used in [grid_util::Direction].
    fn indexed_neighbor(&self, node: &Point, index: i32) -> bool {
        (self.neighbours.get_point(*node) & 1 << (index.rem_euclid(8))) != 0
    }
    fn is_forced(&self, dir: Direction, node: &Point) -> bool {
        let dir_num = dir.num();
        if dir.diagonal() {
            !self.indexed_neighbor(node, 3 + dir_num) || !self.indexed_neighbor(node, 5 + dir_num)
        } else {
            if self.allow_diagonal_move {
                !self.indexed_neighbor(node, 2 + dir_num)
                    || !self.indexed_neighbor(node, dir_num + 6)
            } else {
                !self.indexed_neighbor(node, 3 + dir_num)
                    || !self.indexed_neighbor(node, dir_num + 5) 
                    //|| !self.indexed_neighbor(node, dir_num)
            }
        }
    }
    fn explain_bin_neighborhood(nn: u8) {
        for i in 0..8_i32 {
            let x = nn & (1 << i) != 0;
            let dir = Direction::try_from(i.rem_euclid(8)).unwrap();
            if x {
                println!("\t\t {dir:?}");
            }
        }
    }
    fn pruned_neighborhood<'a>(
        &self,
        dir: Direction,
        node: &'a Point,
    ) -> (impl Iterator<Item = (Point, i32)> + 'a, bool) {
        let dir_num = dir.num();
        let mut n_mask: u8;
        let mut neighbours = self.neighbours.get_point(*node);
        if !self.allow_diagonal_move {
            neighbours &= 0b01010101;
        }
        if DEBUG_PRINT {
            println!("Pruned neighborhood of {node:?} to the {dir:?} of parent.");
            println!("\tNeighbours: {neighbours:#010b}");
            PathingGrid::explain_bin_neighborhood(neighbours);
        }
        let mut forced = false;
        if dir.diagonal() {
            if DEBUG_PRINT {
                println!("\tDiagonal: {dir_num} or {dir:?}");
            }
            n_mask = 0b11000001_u8.rotate_left(dir_num as u32);
            if !self.indexed_neighbor(node, 3 + dir_num) {
                n_mask |= 1 << ((dir_num + 2) % 8);
                forced = true;
            }
            if !self.indexed_neighbor(node, 5 + dir_num) {
                n_mask |= 1 << ((dir_num + 6) % 8);
                forced = true;
            }
        } else {
            if DEBUG_PRINT {
                println!("\tStraight: {dir_num} or {dir:?}");
            }
            n_mask = 0b00000001 << dir_num;
            if self.allow_diagonal_move {
                if !self.indexed_neighbor(node, 2 + dir_num) {
                    n_mask |= 1 << ((dir_num + 1) % 8);
                    forced = true;
                }
                if !self.indexed_neighbor(node, 6 + dir_num) {
                    n_mask |= 1 << ((dir_num + 7) % 8);
                    forced = true;
                }
            } else {
                if !self.indexed_neighbor(node, 3 + dir_num)
                {
                    n_mask |= 1 << ((dir_num + 6) % 8);
                    forced = true;
                }
                if !self.indexed_neighbor(node, 5 + dir_num)
                {
                    n_mask |= 1 << ((dir_num + 2) % 8);
                    forced = true;
                }
                if !self.indexed_neighbor(node, dir_num)
                {
                    n_mask |= 1 << ((dir_num + 6) % 8);
                    n_mask |= 1 << ((dir_num + 2) % 8);
                    forced = true;
                }
            }
        }
        let comb_mask = neighbours & n_mask;
        if DEBUG_PRINT {
            println!("\tForced neighbour mask: {n_mask:#010b}");
            PathingGrid::explain_bin_neighborhood(n_mask);
            println!("\tCombined mask: {comb_mask:#010b}");
            PathingGrid::explain_bin_neighborhood(comb_mask)
        };
        (
            (0..8)
                .step_by(if self.allow_diagonal_move { 1 } else { 2 })
                .filter(move |x| comb_mask & (1 << *x) != 0)
                .map(|d| (node.moore_neighbor(d), 1)),
            forced,
        )
    }

    fn jump<F>(
        &self,
        initial: &Point,
        cost: i32,
        direction: Direction,
        goal: &F,
    ) -> Option<(Point, i32)>
    where
        F: Fn(&Point) -> bool,
    {
        let new_n = *initial + direction;
        if !self.can_move_to(new_n) {
            return None;
        }

        if goal(&new_n) {
            return Some((new_n, cost));
        }
        if self.is_forced(direction, &new_n) {
            return Some((new_n, cost));
        }

        if direction.diagonal()
            && (self.jump(&new_n, 1, direction.x_dir(), goal).is_some()
                || self.jump(&new_n, 1, direction.y_dir(), goal).is_some())
        {
            return Some((new_n, cost));
        }
        if !self.allow_diagonal_move && !direction.diagonal() {
            if !self.can_move_to(new_n + direction) {
                let perp_1 = direction.rotate_ccw(2);
                let perp_2 = direction.rotate_cw(2);
                if self.jump(&new_n, 1, perp_1, goal).is_some()
                    ||self.jump(&new_n, 1, perp_2, goal).is_some()
                {
                    return Some((new_n, cost));
                }
            }
        }
        self.jump(&new_n, cost + 1, direction, goal)
    }
    fn pathfinding_neighborhood(&self, pos: &Point) -> Vec<(Point, i32)> {
        if self.allow_diagonal_move {
            pos.moore_neighborhood()
        } else {
            pos.neumann_neighborhood()
        }
        .into_iter()
        .filter(|p| self.can_move_to(*p))
        .map(|p| (p, 1))
        .collect::<Vec<_>>()
    }
    fn update_neighbours(&mut self, x: i32, y: i32, blocked: bool) {
        let p = Point::new(x, y);
        for i in 0..8 {
            let neighbor = p.moore_neighbor(i);
            if self.in_bounds(neighbor.x, neighbor.y) {
                let ix = (i + 4) % 8;
                let mut n_mask = self.neighbours.get_point(neighbor);
                if blocked {
                    n_mask &= !(1 << ix);
                } else {
                    n_mask |= 1 << ix;
                }
                self.neighbours.set_point(neighbor, n_mask);
            }
        }
    }
    fn jps_neighbours<F>(&self, parent: Option<&Point>, node: &Point, goal: &F) -> Vec<(Point, i32)>
    where
        F: Fn(&Point) -> bool,
    {
        match parent {
            Some(parent_node) => {
                if DEBUG_PRINT {
                    println!("Point: {:?}; Parent: {:?}", node, parent_node);
                }
                let mut succ = vec![];
                let dir = parent_node.dir_obj(node);
                for (n, c) in self.pruned_neighborhood(dir, &node).0 {
                    let dir = node.dir_obj(&n);
                    if let Some((jumped_node, cost)) = self.jump(node, c, dir, goal) {
                        let neighbour_dir = node.dir_obj(&jumped_node);
                        // If improved pruning is enabled, expand any diagonal unforced nodes
                        if self.improved_pruning
                            && dir.diagonal()
                            && !self.is_forced(neighbour_dir, &jumped_node)
                        {
                            // Recursively expand the unforced diagonal node
                            let jump_points =
                                self.jps_neighbours(Some(parent_node), &jumped_node, goal);

                            // Extend the successors with the neighbours of the unforced node, correcting the
                            // cost to include the cost from parent_node to jumped_node
                            succ.extend(jump_points.into_iter().map(|(p, c)| (p, c + cost)));
                        } else {
                            if DEBUG_PRINT {
                                println!("\tJumped node: {:?}", jumped_node);
                            }
                            succ.push((jumped_node, cost));
                        }
                    }
                    else{
                        if DEBUG_PRINT {
                            println!("\tNode went nowhere after jumping: {:?}", n);
                        }
                    }
                }
                if DEBUG_PRINT {
                    println!("\tFinal successors: {:?}", succ);
                }
                succ
            }
            None => {
                // For the starting node, just generate the full normal neighborhood without any pruning or jumping.
                let pf_neighborhood = self.pathfinding_neighborhood(node);
                if DEBUG_PRINT {
                    println!("Initial neighborhood: {:?}", pf_neighborhood);
                }
                pf_neighborhood
            }
        }
    }
    /// Retrieves the component id a given [Point] belongs to.
    pub fn get_component(&self, point: &Point) -> usize {
        self.components.find(self.get_ix_point(point))
    }
    /// Checks if start and goal are on the same component.
    pub fn reachable(&self, start: &Point, goal: &Point) -> bool {
        !self.unreachable(start, goal)
    }

    /// Checks if start and goal are on the same component.
    pub fn unreachable(&self, start: &Point, goal: &Point) -> bool {
        if self.in_bounds(start.x, start.y) && self.in_bounds(goal.x, goal.y) {
            let start_ix = self.get_ix_point(start);
            let goal_ix = self.get_ix_point(goal);
            if self.components.equiv(start_ix, goal_ix) {
                false
            } else {
                // start_ix and goal_ix are not equivalent components
                true
            }
        } else {
            true
        }
    }
    /// Checks if any neighbour of the goal is on the same component as the start.
    pub fn neighbours_unreachable(&self, start: &Point, goal: &Point) -> bool {
        if self.in_bounds(start.x, start.y) && self.in_bounds(goal.x, goal.y) {
            let start_ix = self.get_ix_point(start);
            let neighborhood = if self.allow_diagonal_move {
                goal.moore_neighborhood()
            } else {
                goal.neumann_neighborhood()
            };
            !neighborhood.iter().any(|p| {
                self.in_bounds(p.x, p.y) && self.components.equiv(start_ix, self.get_ix_point(&p))
            })
        } else {
            true
        }
    }
    /// Computes a path from start to goal using JPS. If approximate is [true], then it will
    /// path to one of the neighbours of the goal, which is useful if the goal itself is
    /// blocked. The heuristic used is the [move (Chebyshev) distance](https://en.wikipedia.org/wiki/Chebyshev_distance) or
    /// [Manhattan distance](https://en.wikipedia.org/wiki/Taxicab_geometry) depending on whether diagonal moves are allowed (see [heuristic](Self::heuristic)). This can be
    /// specified by setting [allow_diagonal_move](Self::allow_diagonal_move).
    /// The heuristic will be scaled by [heuristic_factor](Self::heuristic_factor) which can be used to trade optimality for faster solving for many practical problems. In pathfinding language, a factor greater than
    /// 1.0 will make the heuristic [inadmissible](https://en.wikipedia.org/wiki/Admissible_heuristic), a requirement for solution optimality. By default,
    /// the [heuristic_factor](Self::heuristic_factor) is 1.0 which gives optimal solutions.
    pub fn get_path_single_goal(
        &self,
        start: Point,
        goal: Point,
        approximate: bool,
    ) -> Option<Vec<Point>> {
        self.get_waypoints_single_goal(start, goal, approximate)
            .map(|x| waypoints_to_path(x))
    }

    /// Computes a path from the start to one of the given goals and returns the selected goal in addition to the found path. Otherwise behaves similar to [get_path_single_goal](Self::get_path_single_goal).
    pub fn get_path_multiple_goals(
        &self,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        self.get_waypoints_multiple_goals(start, goals)
            .map(|(x, y)| (x, waypoints_to_path(y)))
    }
    /// The raw waypoints (jump points) from which [get_path_multiple_goals](Self::get_path_multiple_goals) makes a path.
    pub fn get_waypoints_multiple_goals(
        &self,
        start: Point,
        goals: Vec<&Point>,
    ) -> Option<(Point, Vec<Point>)> {
        if goals.is_empty() {
            return None;
        }
        let result = astar_jps(
            &start,
            |parent, node| {
                self.jps_neighbours(*parent, node, &|node_pos| goals.contains(&node_pos))
            },
            |point| {
                (goals
                    .iter()
                    .map(|x| self.heuristic(&point, x))
                    .min()
                    .unwrap() as f32
                    * self.heuristic_factor) as i32
            },
            |point| goals.contains(&point),
        );
        result.map(|(v, _c)| (*v.last().unwrap(), v))
    }
    /// The raw waypoints (jump points) from which [get_path_single_goal](Self::get_path_single_goal) makes a path.
    pub fn get_waypoints_single_goal(
        &self,
        start: Point,
        goal: Point,
        approximate: bool,
    ) -> Option<Vec<Point>> {
        if approximate {
            if self.neighbours_unreachable(&start, &goal) {
                // No neigbhours of the goal are reachable from the start
                return None;
            }
            // A neighbour of the goal can be reached, compute a path
            astar_jps(
                &start,
                |parent, node| {
                    self.jps_neighbours(*parent, node, &|node_pos| {
                        self.heuristic(&node_pos, &goal) <= 1
                    })
                },
                |point| (self.heuristic(point, &goal) as f32 * self.heuristic_factor) as i32,
                |point| self.heuristic(point, &goal) <= 1,
            )
        } else {
            if self.unreachable(&start, &goal) {
                return None;
            }
            // The goal is reachable from the start, compute a path
            astar_jps(
                &start,
                |parent, node| self.jps_neighbours(*parent, node, &|node_pos| *node_pos == goal),
                |point| (self.heuristic(point, &goal) as f32 * self.heuristic_factor) as i32,
                |point| *point == goal,
            )
        }
        .map(|(v, _c)| v)
    }
    /// Regenerates the components if they are marked as dirty.
    pub fn update(&mut self) {
        if self.components_dirty {
            // The components are dirty, regenerate them
            self.generate_components();
        }
    }
    /// Generates a new [UnionFind] structure and links up grid neighbours to the same components.
    pub fn generate_components(&mut self) {
        let w = self.grid.width;
        let h = self.grid.height;
        self.components = UnionFind::new(w * h);
        self.components_dirty = false;
        for x in 0..w {
            for y in 0..h {
                if !self.grid.get(x, y) {
                    let parent_ix = self.grid.get_ix(x, y);
                    let point = Point::new(x as i32, y as i32);

                    let neighbours = if self.allow_diagonal_move {
                        vec![
                            Point::new(point.x, point.y + 1),
                            Point::new(point.x, point.y - 1),
                            Point::new(point.x + 1, point.y),
                            Point::new(point.x + 1, point.y - 1),
                            Point::new(point.x + 1, point.y),
                            Point::new(point.x + 1, point.y + 1),
                        ]
                    } else {
                        vec![
                            Point::new(point.x, point.y + 1),
                            Point::new(point.x, point.y - 1),
                            Point::new(point.x + 1, point.y),
                        ]
                    }
                    .into_iter()
                    .filter(|p| self.grid.point_in_bounds(*p) && !self.grid.get_point(*p))
                    .map(|p| self.grid.get_ix(p.x as usize, p.y as usize))
                    .collect::<Vec<usize>>();
                    for ix in neighbours {
                        self.components.union(parent_ix, ix);
                    }
                }
            }
        }
    }
}
impl fmt::Display for PathingGrid {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Grid:")?;
        for y in 0..self.grid.height {
            let values = (0..self.grid.width)
                .map(|x| self.grid.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        writeln!(f, "\nNeighbours:")?;
        for y in 0..self.neighbours.height {
            let values = (0..self.neighbours.width)
                .map(|x| self.neighbours.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        Ok(())
    }
}

impl Grid<bool> for PathingGrid {
    fn new(width: usize, height: usize, default_value: bool) -> Self {
        let mut base_grid = PathingGrid {
            grid: BoolGrid::new(width, height, default_value),
            neighbours: SimpleGrid::new(width, height, 0b11111111),
            components: UnionFind::new(width * height),
            components_dirty: false,
            improved_pruning: true,
            heuristic_factor: 1.0,
            allow_diagonal_move: true,
        };
        // Emulates 'placing' of blocked tile around map border to correctly initialize neighbours
        // and make behaviour of a map bordered by tiles the same as a borderless map.
        for i in -1..=(width as i32) {
            base_grid.update_neighbours(i, -1, true);
            base_grid.update_neighbours(i, height as i32, true);
        }
        for j in -1..=(height as i32) {
            base_grid.update_neighbours(-1, j, true);
            base_grid.update_neighbours(width as i32, j, true);
        }
        base_grid
    }
    fn get(&self, x: usize, y: usize) -> bool {
        self.grid.get(x, y)
    }
    /// Updates a position on the grid. Joins newly connected components and flags the components
    /// as dirty if components are (potentially) broken apart into multiple.
    fn set(&mut self, x: usize, y: usize, blocked: bool) {
        let p = Point::new(x as i32, y as i32);
        if self.grid.get(x, y) != blocked && blocked {
            self.components_dirty = true;
        } else {
            for p in self.get_neighbours(p) {
                self.components.union(
                    self.grid.get_ix(x, y),
                    self.grid.get_ix(p.x as usize, p.y as usize),
                );
            }
        }
        self.update_neighbours(p.x, p.y, blocked);
        self.grid.set(x, y, blocked);
    }
    fn width(&self) -> usize {
        self.grid.width()
    }
    fn height(&self) -> usize {
        self.grid.height()
    }
}

#[cfg(test)]
mod tests {
    use grid_util::Rect;

    use super::*;
    fn visualize_grid(grid: &PathingGrid) {
        let grid = &grid.grid;
        for y in (0..grid.height).rev() {
            for x in 0..grid.width {
                if grid.get(x, y) {
                    print!("#");
                } else {
                    print!(".");
                }
            }
            println!();
        }
    }
    /// Tests whether points are correctly mapped to different connected components
    #[test]
    fn test_component_generation() {
        // Corresponds to the following 3x3 grid:
        //  ___
        // | # |
        // | # |
        //  ___
        let mut path_graph = PathingGrid::new(3, 2, false);
        path_graph.grid.set(1, 0, true);
        path_graph.grid.set(1, 1, true);
        let f_ix = |p| path_graph.get_ix_point(p);
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1, 1);
        let p3 = Point::new(0, 1);
        let p4 = Point::new(2, 0);
        let p1_ix = f_ix(&p1);
        let p2_ix = f_ix(&p2);
        let p3_ix = f_ix(&p3);
        let p4_ix = f_ix(&p4);
        path_graph.generate_components();
        assert!(!path_graph.components.equiv(p1_ix, p2_ix));
        assert!(path_graph.components.equiv(p1_ix, p3_ix));
        assert!(!path_graph.components.equiv(p1_ix, p4_ix));
    }

    #[test]
    fn reachable_with_diagonals() {
        let mut path_graph = PathingGrid::new(3, 2, false);
        path_graph.grid.set(1, 0, true);
        path_graph.grid.set(1, 1, true);
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1, 1);
        let p3 = Point::new(0, 1);
        let p4 = Point::new(2, 0);
        path_graph.generate_components();
        assert!(path_graph.unreachable(&p1, &p2));
        assert!(!path_graph.unreachable(&p1, &p3));
        assert!(path_graph.unreachable(&p1, &p4));
        assert!(!path_graph.neighbours_unreachable(&p1, &p2));
        assert!(path_graph.neighbours_unreachable(&p1, &p4));
    }

    /// Asserts that the optimal 4 step solution is found. Uses improved pruning and allows diagonals.
    #[test]
    fn reachable_without_diagonals() {
        // |S  |
        // | # |
        // |  G|
        //  ___
        let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
        pathing_grid.improved_pruning = false;
        pathing_grid.allow_diagonal_move = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        assert!(pathing_grid.reachable(&start, &end));
    }

    /// Asserts that the optimal 4 step solution is found. Does not allow diagonals.
    #[test]
    fn solve_simple_problem() {
        let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
        // pathing_grid.improved_pruning = false;
        pathing_grid.allow_diagonal_move = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        let path = pathing_grid
            .get_path_single_goal(start, end, false)
            .unwrap();
        // The shortest path takes 5 steps
        assert!(path.len() == 5);
    }

    /// Asserts that the optimal 4 step solution is found. Allows diagonals.
    #[test]
    fn solve_simple_problem_diagonals() {
        let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
        pathing_grid.improved_pruning = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        let path = pathing_grid
            .get_path_single_goal(start, end, false)
            .unwrap();
        // The shortest path takes 4 steps
        assert!(path.len() == 4);
    }

    /// Asserts that the optimal 4 step solution is found. Uses improved pruning and allows diagonals (default behaviour).
    #[test]
    fn solve_simple_problem_improved_pruning() {
        let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        let path = pathing_grid
            .get_path_single_goal(start, end, false)
            .unwrap();
        // The shortest path takes 4 steps
        assert!(path.len() == 4);
    }

    #[test]
    fn test_multiple_goals() {
        let mut pathing_grid: PathingGrid = PathingGrid::new(5, 5, false);
        pathing_grid.allow_diagonal_move = false;
        pathing_grid.improved_pruning = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let goal_1 = Point::new(4, 4);
        let goal_2 = Point::new(2, 2);
        let goals = vec![&goal_1, &goal_2];
        let (selected_goal, path) = pathing_grid.get_path_multiple_goals(start, goals).unwrap();
        assert_eq!(selected_goal, Point::new(2, 2));
        assert!(path.len() == 5);
    }

    #[test]
    fn test_multiple_goals_diagonals() {
        let mut pathing_grid: PathingGrid = PathingGrid::new(5, 5, false);
        pathing_grid.improved_pruning = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let goal_1 = Point::new(4, 4);
        let goal_2 = Point::new(2, 2);
        let goals = vec![&goal_1, &goal_2];
        let (selected_goal, path) = pathing_grid.get_path_multiple_goals(start, goals).unwrap();
        assert_eq!(selected_goal, Point::new(2, 2));
        assert!(path.len() == 4);
    }

    #[test]
    fn test_multiple_goals_improved_pruning() {
        let mut pathing_grid: PathingGrid = PathingGrid::new(5, 5, false);
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let goal_1 = Point::new(4, 4);
        let goal_2 = Point::new(2, 2);
        let goals = vec![&goal_1, &goal_2];
        let (selected_goal, path) = pathing_grid.get_path_multiple_goals(start, goals).unwrap();
        assert_eq!(selected_goal, Point::new(2, 2));
        // The shortest path takes 4 steps
        assert!(path.len() == 4);
    }

    #[test]
    fn test_diagonal_switch_reachable() {
        // Tests the effect of allowing diagonals in solving the following 2x2 grid:
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
        let start = Point::new(0, 0);
        let end = Point::new(1, 1);
        assert!(pathing_grid.unreachable(&start, &end));
        assert!(pathing_grid_diag.reachable(&start, &end));
    }

    #[test]
    fn test_diagonal_switch_path() {
        // Tests the effect of allowing diagonals in solving the following 2x2 grid:
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
        let start = Point::new(0, 0);
        let goal = Point::new(1, 1);
        let path = pathing_grid.get_path_single_goal(start, goal, false);
        let path_diag = pathing_grid_diag.get_path_single_goal(start, goal, false);
        assert!(path.is_none());
        assert!(path_diag.is_some());
    }
}
