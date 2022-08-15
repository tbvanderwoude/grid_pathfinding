//! # grid_pathfinding
//!
//! A complete grid-based pathfinding system. Implements
//! [Jump Point Search](https://en.wikipedia.org/wiki/Jump_point_search) for speedy optimal
//! pathfinding. Pre-computes
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

/// [PathingGrid] maintains information about components and neighbours in addition to the raw
/// [bool] grid values in the [BoolGrid] that determine whether a space is occupied ([true]) or
/// empty ([false]). I
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
    /// Regenerates the components if they are marked as dirty.
    pub fn update(&mut self) {
        if self.components_dirty {
            info!("Components are dirty: regenerating components");
            self.generate_components();
        }
    }
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
    fn unreachable(&self, start: &Point, goal: &Point) -> bool {
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
            if !self.indexed_neighbor(node, dir_num + 6) {
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
    pub fn get_component(&self, point: &Point) -> usize {
        self.components.find(self.get_ix_point(point))
    }
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
    pub fn get_path_multiple_goals(
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
    pub fn get_path_single_goal(
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
            astar_jps(
                &start,
                |&parent, node| self.jps_neighbours(parent, node, &|node_pos| *node_pos == goal),
                |&point| (point.move_distance(&goal) as f32 * HEURISTIC_FACTOR) as i32,
                |node_pos| *node_pos == goal,
            )
        }
        .map(|(v, _c)| v)
    }
}
impl fmt::Display for PathingGrid {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for y in 0..self.grid.height {
            let values = (0..self.grid.width)
                .map(|x| self.grid.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        Ok(())
    }
}

impl Grid<bool> for PathingGrid {
    fn new(width: usize, height: usize, default_value: bool) -> Self {
        PathingGrid {
            grid: BoolGrid::new(width, height, default_value),
            neighbours: SimpleGrid::new(width, height, 0),
            components: UnionFind::new(width * height),
            components_dirty: false,
        }
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
        for i in 0..=8 {
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
        let mut path_graph = PathingGrid::new(3, 4,true);
        path_graph.grid.set(1, 1, false);
        path_graph.generate_components();
        assert!(!path_graph.components.equiv(0, 4))
    }
}
