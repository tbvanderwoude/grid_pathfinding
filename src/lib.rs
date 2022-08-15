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
use log::info;
use petgraph::unionfind::UnionFind;

use crate::astar_jps::astar_jps;
use core::fmt;
use std::collections::VecDeque;

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
pub struct PathingGrid {
    pub grid: BoolGrid,
    pub neighbours: SimpleGrid<u8>,
    pub components: UnionFind<usize>,
    pub components_dirty: bool,
}

const IMPROVED_PRUNING: bool = true;
const HEURISTIC_FACTOR: f32 = 1.2;

impl Default for PathingGrid {
    fn default() -> PathingGrid {
        PathingGrid {
            grid: BoolGrid::default(),
            neighbours: SimpleGrid::default(),
            components: UnionFind::new(0),
            components_dirty: false,
        }
    }
}
impl PathingGrid {
    fn get_neighbours(&self, point: Point) -> Vec<Point> {
        point
            .moore_neighborhood()
            .into_iter()
            .filter(|p| self.can_move_to(*p))
            .collect::<Vec<Point>>()
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
            !self.indexed_neighbor(node, 2 + dir_num) || !self.indexed_neighbor(node, dir_num + 6)
        }
    }
    fn pruned_neighborhood(&self, dir: Direction, node: &Point) -> (Vec<(Point, i32)>, bool) {
        let dir_num = dir.num();
        let mut n_mask: u8;
        let neighbours = self.neighbours.get_point(*node);
        let mut forced = false;
        if dir.diagonal() {
            n_mask = 131_u8.rotate_left(dir_num as u32);
            if !self.indexed_neighbor(node, 3 + dir_num) {
                n_mask |= 1 << ((dir_num + 2) % 8);
                forced = true;
            }
            if !self.indexed_neighbor(node, 5 + dir_num) {
                n_mask |= 1 << ((dir_num + 6) % 8);
                forced = true;
            }
        } else {
            n_mask = 1 << dir_num;
            if !self.indexed_neighbor(node, 2 + dir_num) {
                n_mask |= 1 << ((dir_num + 1) % 8);
                forced = true;
            }
            if !self.indexed_neighbor(node, 6 + dir_num) {
                n_mask |= 1 << ((dir_num + 7) % 8);
                forced = true;
            }
        }
        let comb_mask = neighbours & n_mask;
        (
            (0..8)
                .filter(|x| comb_mask & (1 << *x) != 0)
                .map(|d| (node.moore_neighbor(d), 1))
                .collect::<Vec<(Point, i32)>>(),
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
        self.jump(&new_n, cost + 1, direction, goal)
    }
    fn pathfinding_neighborhood(&self, pos: &Point) -> Vec<(Point, i32)> {
        pos.moore_neighborhood()
            .into_iter()
            .filter(|&position| self.can_move_to(position))
            .map(|p| (p, 1))
            .collect::<Vec<_>>()
    }
    fn update_neighbours(&mut self, x: i32, y: i32, blocked: bool) {
        let p = Point::new(x, y);
        for i in 0..8 {
            let neighbor = p.moore_neighbor(i);
            if self.in_bounds(neighbor.x, neighbor.y) {
                info!("Yes");
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
                let mut succ = vec![];
                let dir = parent_node.dir_obj(node);
                for (n, c) in &self.pruned_neighborhood(dir, &node).0 {
                    let dir = node.dir_obj(&n);
                    if let Some((jumped_node, cost)) = self.jump(node, *c, dir, goal) {
                        let neighbour_dir = node.dir_obj(&jumped_node);
                        if IMPROVED_PRUNING
                            && dir.diagonal()
                            && !self.is_forced(neighbour_dir, &jumped_node)
                        {
                            let mut jump_points =
                                self.jps_neighbours(Some(parent_node), &jumped_node, goal);
                            succ.append(&mut jump_points);
                        }
                        {
                            succ.push((jumped_node, cost));
                        }
                    }
                }
                succ
            }
            None => self.pathfinding_neighborhood(node),
        }
    }
    /// Retrieves the component id a given [Point] belongs to.
    pub fn get_component(&self, point: &Point) -> usize {
        self.components.find(self.get_ix_point(point))
    }
    /// Checks if start and goal are on the same component.
    pub fn unreachable(&self, start: &Point, goal: &Point) -> bool {
        if self.in_bounds(start.x, start.y) && self.in_bounds(goal.x, goal.y) {
            let start_ix = self.get_ix_point(start);
            let goal_ix = self.get_ix_point(goal);
            if self.components.equiv(start_ix, goal_ix) {
                false
            } else {
                info!("{} and {} are not equivalent components", start_ix, goal_ix);
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
            !goal.moore_neighborhood().iter().any(|p| {
                self.in_bounds(p.x, p.y) && self.components.equiv(start_ix, self.get_ix_point(&p))
            })
        } else {
            true
        }
    }
    /// Computes a path from start to goal using JPS. If approximate is [true], then it will
    /// path to one of the neighbours of the goal, which is useful if goal itself is
    /// blocked. The heuristic used is the
    /// [Chebyshev distance](https://en.wikipedia.org/wiki/Chebyshev_distance).
    pub fn get_path_single_goal(
        &self,
        start: Point,
        goal: Point,
        approximate: bool,
    ) -> Option<Vec<Point>> {
        self.get_waypoints_single_goal(start, goal, approximate)
            .map(|x| waypoints_to_path(x))
    }

    /// Computes a path from start to one of the given goals. This is done by taking the
    /// [Chebyshev distance](https://en.wikipedia.org/wiki/Chebyshev_distance)
    /// to the closest goal as heuristic value.
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
            |&parent, node| {
                self.jps_neighbours(parent, node, &|node_pos| goals.contains(&node_pos))
            },
            |&point| {
                (goals.iter().map(|x| point.move_distance(*x)).min().unwrap() as f32
                    * HEURISTIC_FACTOR) as i32
            },
            |node_pos| goals.contains(&node_pos),
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
                info!("No neighbours of {} are reachable from {}", goal, start);
                return None;
            }
            info!(
                "Neighborhood of {} is reachable from {}, computing path",
                goal, start
            );
            astar_jps(
                &start,
                |&parent, node| {
                    self.jps_neighbours(parent, node, &|node_pos| {
                        node_pos.move_distance(&goal) <= 1
                    })
                },
                |&point| (point.move_distance(&goal) as f32 * HEURISTIC_FACTOR) as i32,
                |node_pos| node_pos.move_distance(&goal) <= 1,
            )
        } else {
            if self.unreachable(&start, &goal) {
                info!("{} is not reachable from {}", start, goal);
                return None;
            }
            info!("{} is reachable from {}, computing path", goal, start);
            astar_jps(
                &start,
                |&parent, node| self.jps_neighbours(parent, node, &|node_pos| *node_pos == goal),
                |&point| (point.move_distance(&goal) as f32 * HEURISTIC_FACTOR) as i32,
                |node_pos| *node_pos == goal,
            )
        }
        .map(|(v, _c)| v)
    }
    /// Regenerates the components if they are marked as dirty.
    pub fn update(&mut self) {
        if self.components_dirty {
            info!("Components are dirty: regenerating components");
            self.generate_components();
        }
    }
    /// Generates a new [UnionFind] structure and links up grid neighbours to the same components.
    pub fn generate_components(&mut self) {
        info!("Generating connected components");
        let w = self.grid.width;
        let h = self.grid.height;
        self.components = UnionFind::new(w * h);
        self.components_dirty = false;
        for x in 0..w {
            for y in 0..h {
                if !self.grid.get(x, y) {
                    let parent_ix = self.grid.get_ix(x, y);
                    let point = Point::new(x as i32, y as i32);
                    let neighbours = vec![
                        Point::new(point.x, point.y + 1),
                        Point::new(point.x + 1, point.y),
                        Point::new(point.x + 1, point.y + 1),
                    ]
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
            neighbours: SimpleGrid::new(width, height, 255),
            components: UnionFind::new(width * height),
            components_dirty: false,
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
    use super::*;
    #[test]
    fn test_component_generation() {
        let mut path_graph = PathingGrid::new(3, 4, true);
        path_graph.grid.set(1, 1, false);
        path_graph.generate_components();
        assert!(!path_graph.components.equiv(0, 4))
    }
}
