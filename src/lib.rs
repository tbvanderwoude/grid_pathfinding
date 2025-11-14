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
use grid_util::grid::{BoolGrid, SimpleValueGrid, ValueGrid};
use grid_util::point::Point;
use petgraph::unionfind::UnionFind;
use smallvec::SmallVec;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

const EQUAL_EDGE_COST: bool = false;
const ALLOW_CORNER_CUTTING: bool = false;
const GRAPH_PRUNING: bool = true;
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
pub struct Pathfinder {
    pub grid: BoolGrid,
    pub neighbours: SimpleValueGrid<u8>,
    pub jump_point: SimpleValueGrid<u8>,
    pub components: UnionFind<usize>,
    pub components_dirty: bool,
    pub heuristic_factor: f32,
    pub improved_pruning: bool,
    pub allow_diagonal_move: bool,
    context: Arc<Mutex<SearchContext<Point, i32>>>,
}

impl Default for Pathfinder {
    fn default() -> Pathfinder {
        let mut grid = Pathfinder {
            grid: BoolGrid::default(),
            neighbours: SimpleValueGrid::default(),
            jump_point: SimpleValueGrid::default(),
            components: UnionFind::new(0),
            components_dirty: false,
            improved_pruning: true,
            heuristic_factor: 1.0,
            allow_diagonal_move: true,
            context: Arc::new(Mutex::new(SearchContext::new())),
        };
        grid.initialize();
        grid
    }
}
impl Pathfinder {
    fn neighborhood_points(&self, point: &Point) -> SmallVec<[Point; 8]> {
        if self.allow_diagonal_move {
            point.moore_neighborhood_smallvec()
        } else {
            point.neumann_neighborhood_smallvec()
        }
    }
    fn neighborhood_points_and_cost(
        &self,
        pos: &Point,
    ) -> SmallVec<[(Point, i32); N_SMALLVEC_SIZE]> {
        self.neighborhood_points(pos)
            .into_iter()
            .filter(|p| self.can_move_to(*p, *pos))
            // See comment in pruned_neighborhood about cost calculation
            .map(move |p| (p, (pos.dir_obj(&p).num() % 2) * (D - C) + C))
            .collect::<SmallVec<[_; N_SMALLVEC_SIZE]>>()
    }
    /// Uses C as cost for cardinal (straight) moves and D for diagonal moves.
    pub fn heuristic(&self, p1: &Point, p2: &Point) -> i32 {
        if self.allow_diagonal_move {
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
    fn can_move_to(&self, pos: Point, start: Point) -> bool {
        if ALLOW_CORNER_CUTTING {
            self.can_move_to_simple(pos)
        } else {
            debug_assert!((start.x - pos.x).abs() <= 1 && (start.y - pos.y).abs() <= 1);
            self.can_move_to_simple(pos)
                && (!self.grid.get_point(Point::new(start.x, pos.y))
                    && !self.grid.get_point(Point::new(pos.x, start.y)))
        }
    }
    fn can_move_to_simple(&self, pos: Point) -> bool {
        self.point_in_bounds(pos) && !self.grid.get_point(pos)
    }
    fn in_bounds(&self, x: i32, y: i32) -> bool {
        self.grid.index_in_bounds(x, y)
    }
    /// The neighbour indexing used here corresponds to that used in [grid_util::Direction].
    fn indexed_neighbor(&self, node: &Point, index: i32) -> bool {
        (self.neighbours.get_point(*node) & 1 << (index.rem_euclid(8))) != 0
    }
    fn is_forced(&self, dir: Direction, node: &Point) -> bool {
        let dir_num = dir.num();
        self.jump_point.get_point(*node) & (1 << dir_num) != 0
    }

    fn forced_mask(&self, node: &Point) -> u8 {
        let mut forced_mask: u8 = 0;
        for dir_num in 0..8 {
            if dir_num % 2 == 1 {
                if !self.indexed_neighbor(node, 3 + dir_num)
                    || !self.indexed_neighbor(node, 5 + dir_num)
                {
                    forced_mask |= 1 << dir_num;
                }
            } else {
                if !self.indexed_neighbor(node, 2 + dir_num)
                    || !self.indexed_neighbor(node, 6 + dir_num)
                {
                    forced_mask |= 1 << dir_num;
                }
            };
        }
        forced_mask
    }

    fn pruned_neighborhood<'a>(
        &self,
        dir: Direction,
        node: &'a Point,
    ) -> impl Iterator<Item = (Point, i32)> + 'a {
        let dir_num = dir.num();
        let mut n_mask: u8;
        let mut neighbours = self.neighbours.get_point(*node);
        if !self.allow_diagonal_move {
            neighbours &= 0b01010101;
            n_mask = 0b01000101_u8.rotate_left(dir_num as u32);
        } else if dir.diagonal() {
            n_mask = 0b10000011_u8.rotate_left(dir_num as u32);
            if !self.indexed_neighbor(node, 3 + dir_num) {
                n_mask |= 1 << ((dir_num + 2) % 8);
            }
            if !self.indexed_neighbor(node, 5 + dir_num) {
                n_mask |= 1 << ((dir_num + 6) % 8);
            }
        } else {
            n_mask = 0b00000001 << dir_num;
            if !self.indexed_neighbor(node, 2 + dir_num) {
                n_mask |= 1 << ((dir_num + 1) % 8);
            }
            if !self.indexed_neighbor(node, 6 + dir_num) {
                n_mask |= 1 << ((dir_num + 7) % 8);
            }
        }
        let comb_mask = neighbours & n_mask;
        (0..8)
            .step_by(if self.allow_diagonal_move { 1 } else { 2 })
            .filter(move |x| comb_mask & (1 << *x) != 0)
            // (dir_num % 2) * (D-C) + C)
            // is an optimized version without a conditional of
            // if dir.diagonal() {D} else {C}
            .map(move |d| (node.moore_neighbor(d), (dir_num % 2) * (D - C) + C))
    }

    /// Straight jump in a cardinal direction.
    fn jump_straight<F>(
        &self,
        mut initial: Point,
        mut cost: i32,
        direction: Direction,
        goal: &F,
    ) -> Option<(Point, i32)>
    where
        F: Fn(&Point) -> bool,
    {
        debug_assert!(!direction.diagonal());
        loop {
            initial = initial + direction;
            if !self.can_move_to_simple(initial) {
                return None;
            }

            if goal(&initial) || self.is_forced(direction, &initial) {
                return Some((initial, cost));
            }

            // Straight jumps always take cardinal cost
            cost += C;
        }
    }

    /// Performs the jumping of node neighbours, skipping over unnecessary nodes until a goal or a forced node is found.
    fn jump<F>(
        &self,
        mut initial: Point,
        mut cost: i32,
        direction: Direction,
        goal: &F,
    ) -> Option<(Point, i32)>
    where
        F: Fn(&Point) -> bool,
    {
        let mut new_initial: Point;
        loop {
            new_initial = initial + direction;
            if !self.can_move_to(new_initial, initial) {
                return None;
            }
            initial = new_initial;

            if goal(&initial) || self.is_forced(direction, &initial) {
                return Some((initial, cost));
            }
            if direction.diagonal()
                && (self
                    .jump_straight(initial, 1, direction.x_dir(), goal)
                    .is_some()
                    || self
                        .jump_straight(initial, 1, direction.y_dir(), goal)
                        .is_some())
            {
                return Some((initial, cost));
            }

            // When using a 4-neighborhood (specified by setting allow_diagonal_move to false),
            // jumps perpendicular to the direction are performed. This is necessary to not miss the
            // goal when passing by.
            if !self.allow_diagonal_move {
                let perp_1 = direction.rotate_ccw(2);
                let perp_2 = direction.rotate_cw(2);
                if self.jump_straight(initial, 1, perp_1, goal).is_some()
                    || self.jump_straight(initial, 1, perp_2, goal).is_some()
                {
                    return Some((initial, cost));
                }
            }

            // See comment in pruned_neighborhood about cost calculation
            cost += (direction.num() % 2) * (D - C) + C;
        }
    }

    /// Updates the neighbours grid after changing the grid.
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
    fn jps_neighbours<F>(
        &self,
        parent: Option<&Point>,
        node: &Point,
        goal: &F,
    ) -> SmallVec<[(Point, i32); N_SMALLVEC_SIZE]>
    where
        F: Fn(&Point) -> bool,
    {
        match parent {
            Some(parent_node) => {
                let mut succ = SmallVec::new();
                let dir = parent_node.dir_obj(node);
                for (n, c) in self.pruned_neighborhood(dir, node) {
                    let dir = node.dir_obj(&n);
                    // Jumps the neighbor, skipping over unnecessary nodes.
                    if let Some((jumped_node, cost)) = self.jump(*node, c, dir, goal) {
                        // If improved pruning is enabled, expand any diagonal unforced nodes
                        if self.improved_pruning
                            && dir.diagonal()
                            && !goal(&jumped_node)
                            && !self.is_forced(dir, &jumped_node)
                        {
                            // Recursively expand the unforced diagonal node
                            let jump_points = self.jps_neighbours(parent, &jumped_node, goal);

                            // Extend the successors with the neighbours of the unforced node, correcting the
                            // cost to include the cost from parent_node to jumped_node
                            succ.extend(jump_points.into_iter().map(|(p, c)| (p, c + cost)));
                        } else {
                            succ.push((jumped_node, cost));
                        }
                    }
                }
                succ
            }
            None => {
                // For the starting node, just generate the full normal neighborhood without any pruning or jumping.
                self.neighborhood_points_and_cost(node)
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

    /// Checks if start and goal are not on the same component.
    pub fn unreachable(&self, start: &Point, goal: &Point) -> bool {
        if self.in_bounds(start.x, start.y) && self.in_bounds(goal.x, goal.y) {
            let start_ix = self.get_ix_point(start);
            let goal_ix = self.get_ix_point(goal);
            !self.components.equiv(start_ix, goal_ix)
        } else {
            true
        }
    }

    /// Checks if any neighbour of the goal is on the same component as the start.
    pub fn neighbours_reachable(&self, start: &Point, goal: &Point) -> bool {
        if self.in_bounds(start.x, start.y) && self.in_bounds(goal.x, goal.y) {
            let start_ix = self.get_ix_point(start);
            let neighborhood = self.neighborhood_points(goal);
            neighborhood.iter().any(|p| {
                self.in_bounds(p.x, p.y) && self.components.equiv(start_ix, self.get_ix_point(p))
            })
        } else {
            true
        }
    }

    /// Checks if every neighbour of the goal is on a different component as the start.
    pub fn neighbours_unreachable(&self, start: &Point, goal: &Point) -> bool {
        if self.in_bounds(start.x, start.y) && self.in_bounds(goal.x, goal.y) {
            let start_ix = self.get_ix_point(start);
            let neighborhood = self.neighborhood_points(goal);
            neighborhood.iter().all(|p| {
                !self.in_bounds(p.x, p.y) || !self.components.equiv(start_ix, self.get_ix_point(p))
            })
        } else {
            true
        }
    }
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
    pub fn get_path_single_goal(
        &self,
        start: Point,
        goal: Point,
        approximate: bool,
    ) -> Option<Vec<Point>> {
        self.get_waypoints_single_goal(start, goal, approximate)
            .map(waypoints_to_path)
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
        let mut ct = self.context.lock().unwrap();
        let result = ct.astar_jps(
            &start,
            |parent, node| {
                if GRAPH_PRUNING {
                    self.jps_neighbours(*parent, node, &|node_pos| goals.contains(&node_pos))
                } else {
                    self.neighborhood_points_and_cost(node)
                }
            },
            |point| {
                (goals
                    .iter()
                    .map(|x| self.heuristic(point, x))
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
            // Check if start and one of the goal neighbours are on the same connected component.
            if self.neighbours_unreachable(&start, &goal) {
                // No neigbhours of the goal are reachable from the start
                return None;
            }
            // A neighbour of the goal can be reached, compute a path
            let mut ct = self.context.lock().unwrap();
            ct.astar_jps(
                &start,
                |parent, node| {
                    if GRAPH_PRUNING {
                        self.jps_neighbours(*parent, node, &|node_pos| {
                            self.heuristic(node_pos, &goal) <= if EQUAL_EDGE_COST { 1 } else { 99 }
                        })
                    } else {
                        self.neighborhood_points_and_cost(node)
                    }
                },
                |point| (self.heuristic(point, &goal) as f32 * self.heuristic_factor) as i32,
                |point| self.heuristic(point, &goal) <= if EQUAL_EDGE_COST { 1 } else { 99 },
            )
        } else {
            // Check if start and goal are on the same connected component.
            if self.unreachable(&start, &goal) {
                return None;
            }
            // The goal is reachable from the start, compute a path
            let mut ct = self.context.lock().unwrap();
            ct.astar_jps(
                &start,
                |parent, node| {
                    if GRAPH_PRUNING {
                        self.jps_neighbours(*parent, node, &|node_pos| *node_pos == goal)
                    } else {
                        self.neighborhood_points_and_cost(node)
                    }
                },
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

    pub fn update_all_neighbours(&mut self) {
        for x in 0..self.width() as i32 {
            for y in 0..self.height() as i32 {
                self.update_neighbours(x, y, self.get(x, y));
            }
        }
    }
    pub fn set_jumppoints(&mut self, point: Point) {
        let value = self.forced_mask(&point);
        self.jump_point.set_point(point, value);
    }
    pub fn fix_jumppoints(&mut self, point: Point) {
        self.set_jumppoints(point);
        for p in self.neighborhood_points(&point) {
            if self.point_in_bounds(p) {
                self.set_jumppoints(p);
            }
        }
    }

    /// Performs the full jump point precomputation
    pub fn set_all_jumppoints(&mut self) {
        for x in 0..self.width() {
            for y in 0..self.height() {
                self.set_jumppoints(Point::new(x as i32, y as i32));
            }
        }
    }

    pub fn initialize(&mut self) {
        // Emulates 'placing' of blocked tile around map border to correctly initialize neighbours
        // and make behaviour of a map bordered by tiles the same as a borderless map.
        for i in -1..=(self.width() as i32) {
            self.update_neighbours(i, -1, true);
            self.update_neighbours(i, self.height() as i32, true);
        }
        for j in -1..=(self.height() as i32) {
            self.update_neighbours(-1, j, true);
            self.update_neighbours(self.width() as i32, j, true);
        }
        self.update_all_neighbours();
        self.set_all_jumppoints();
    }

    /// Generates a new [UnionFind] structure and links up grid neighbours to the same components.
    pub fn generate_components(&mut self) {
        let w = self.grid.width;
        let h = self.grid.height;
        self.components = UnionFind::new(w * h);
        self.components_dirty = false;
        for x in 0..w as i32 {
            for y in 0..h as i32 {
                if !self.grid.get(x, y) {
                    let point = Point::new(x, y);
                    let parent_ix = self.grid.get_ix_point(&point);

                    if self.allow_diagonal_move {
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
                    .filter(|p| self.can_move_to(*p, point))
                    .collect::<Vec<_>>()
                    .iter()
                    .for_each(|p| {
                        let ix = self.grid.get_ix_point(&p);
                        self.components.union(parent_ix, ix);
                    });
                }
            }
        }
    }
}
impl fmt::Display for Pathfinder {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Grid:")?;
        for y in 0..self.grid.height as i32 {
            let values = (0..self.grid.width as i32)
                .map(|x| self.grid.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        writeln!(f, "\nNeighbours:")?;
        for y in 0..self.neighbours.height as i32 {
            let values = (0..self.neighbours.width as i32)
                .map(|x| self.neighbours.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        Ok(())
    }
}

impl ValueGrid<bool> for Pathfinder {
    fn new(width: usize, height: usize, default_value: bool) -> Self {
        let mut base_grid = Pathfinder {
            grid: BoolGrid::new(width, height, default_value),
            jump_point: SimpleValueGrid::new(width, height, 0b00000000),
            neighbours: SimpleValueGrid::new(width, height, 0b11111111),
            components: UnionFind::new(width * height),
            components_dirty: false,
            improved_pruning: true,
            heuristic_factor: 1.0,
            allow_diagonal_move: true,
            context: Arc::new(Mutex::new(SearchContext::new())),
        };
        base_grid.initialize();
        base_grid
    }
    fn get(&self, x: i32, y: i32) -> bool {
        self.grid.get(x, y)
    }
    /// Updates a position on the grid. Joins newly connected components and flags the components
    /// as dirty if components are (potentially) broken apart into multiple.
    fn set(&mut self, x: i32, y: i32, blocked: bool) {
        let p = Point::new(x, y);
        if self.grid.get(x, y) != blocked && blocked {
            self.components_dirty = true;
        } else {
            let p_ix = self.grid.compute_ix(x, y);
            for n in self.neighborhood_points(&p) {
                if self.can_move_to(n, p) {
                    self.components.union(p_ix, self.grid.get_ix_point(&n));
                }
            }
        }
        self.update_neighbours(p.x, p.y, blocked);
        self.grid.set(x, y, blocked);
        self.fix_jumppoints(p);
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

    /// Tests whether points are correctly mapped to different connected components
    #[test]
    fn test_component_generation() {
        // Corresponds to the following 3x3 grid:
        //  ___
        // | # |
        // | # |
        //  ___
        let mut path_graph = Pathfinder::new(3, 2, false);
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
        let mut path_graph = Pathfinder::new(3, 2, false);
        path_graph.grid.set(1, 0, true);
        path_graph.grid.set(1, 1, true);
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1, 0);
        let p3 = Point::new(0, 1);
        let p4 = Point::new(2, 0);
        path_graph.generate_components();
        assert!(path_graph.unreachable(&p1, &p2));
        assert!(!path_graph.unreachable(&p1, &p3));
        assert!(path_graph.unreachable(&p1, &p4));
        assert!(!path_graph.neighbours_unreachable(&p1, &p2));
        assert!(path_graph.neighbours_unreachable(&p1, &p4));
    }

    /// Asserts that the two corners are connected on a 4-grid.
    #[test]
    fn reachable_without_diagonals() {
        // |S  |
        // | # |
        // |  G|
        //  ___
        let mut pathing_grid: Pathfinder = Pathfinder::new(3, 3, false);
        pathing_grid.improved_pruning = false;
        pathing_grid.allow_diagonal_move = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        assert!(pathing_grid.reachable(&start, &end));
    }

    /// Asserts that the case in which start and goal are equal is handled correctly.
    #[test]
    fn equal_start_goal() {
        for (allow_diag, pruning) in [(false, false), (true, false), (true, true)] {
            let mut pathing_grid: Pathfinder = Pathfinder::new(1, 1, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.generate_components();
            let start = Point::new(0, 0);
            let path = pathing_grid
                .get_path_single_goal(start, start, false)
                .unwrap();
            assert!(path.len() == 1);
        }
    }

    /// Asserts that the optimal 4 step solution is found.
    #[test]
    fn solve_simple_problem() {
        for (allow_diag, pruning, expected) in
            [(false, false, 5), (true, false, 4), (true, true, 4)]
        {
            let mut pathing_grid: Pathfinder = Pathfinder::new(3, 3, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.set(1, 1, true);
            pathing_grid.generate_components();
            let start = Point::new(0, 0);
            let end = Point::new(2, 2);
            let path = pathing_grid
                .get_path_single_goal(start, end, false)
                .unwrap();
            assert!(path.len() == expected);
        }
    }

    #[test]
    fn test_multiple_goals() {
        for (allow_diag, pruning, expected) in
            [(false, false, 7), (true, false, 5), (true, true, 5)]
        {
            let mut pathing_grid: Pathfinder = Pathfinder::new(5, 5, false);
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.set(1, 1, true);
            pathing_grid.generate_components();
            let start = Point::new(0, 0);
            let goal_1 = Point::new(4, 4);
            let goal_2 = Point::new(3, 3);
            let goals = vec![&goal_1, &goal_2];
            let (selected_goal, path) = pathing_grid.get_path_multiple_goals(start, goals).unwrap();
            assert_eq!(selected_goal, Point::new(3, 3));
            assert!(path.len() == expected);
        }
    }

    #[test]
    fn test_complex() {
        for (allow_diag, pruning, expected) in
            [(false, false, 15), (true, false, 10), (true, true, 10)]
        {
            let mut pathing_grid: Pathfinder = Pathfinder::new(10, 10, false);
            pathing_grid.set_rect(Rect::new(1, 1, 1, 1), true);
            pathing_grid.set_rect(Rect::new(5, 0, 1, 1), true);
            pathing_grid.set_rect(Rect::new(0, 5, 1, 1), true);
            pathing_grid.set_rect(Rect::new(8, 8, 1, 1), true);
            // pathing_grid.improved_pruning = false;
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.generate_components();
            let start = Point::new(0, 0);
            let end = Point::new(7, 7);
            let path = pathing_grid
                .get_path_single_goal(start, end, false)
                .unwrap();
            assert!(path.len() == expected);
        }
    }
    #[test]
    fn test_complex_waypoints() {
        for (allow_diag, pruning, expected) in
            [(false, false, 11), (true, false, 7), (true, true, 5)]
        {
            let mut pathing_grid: Pathfinder = Pathfinder::new(10, 10, false);
            pathing_grid.set_rect(Rect::new(1, 1, 1, 1), true);
            pathing_grid.set_rect(Rect::new(5, 0, 1, 1), true);
            pathing_grid.set_rect(Rect::new(0, 5, 1, 1), true);
            pathing_grid.set_rect(Rect::new(8, 8, 1, 1), true);
            // pathing_grid.improved_pruning = false;
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.generate_components();
            let start = Point::new(0, 0);
            let end = Point::new(7, 7);
            let path = pathing_grid
                .get_waypoints_single_goal(start, end, false)
                .unwrap();
            assert!(path.len() == expected);
        }
    }

    // Tests whether allowing diagonals has the expected effect on diagonal reachability in a minimal setting.
    #[test]
    fn test_diagonal_switch_reachable() {
        //  ___
        // | #|
        // |# |
        //  __
        let mut pathing_grid: Pathfinder = Pathfinder::new(2, 2, true);
        pathing_grid.allow_diagonal_move = false;
        let mut pathing_grid_diag: Pathfinder = Pathfinder::new(2, 2, true);
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

    // Tests whether allowing diagonals has the expected effect on path existence in a minimal setting.
    #[test]
    fn test_diagonal_switch_path() {
        //  ___
        // | #|
        // |# |
        //  __
        let mut pathing_grid: Pathfinder = Pathfinder::new(2, 2, true);
        pathing_grid.allow_diagonal_move = false;
        let mut pathing_grid_diag: Pathfinder = Pathfinder::new(2, 2, true);
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
