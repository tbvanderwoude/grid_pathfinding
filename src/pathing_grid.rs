use crate::astar_jps::DefaultSearchContext;

use super::*;
use core::fmt;
use grid_util::grid::{BoolGrid, ValueGrid};
use grid_util::point::Point;
use petgraph::unionfind::UnionFind;
use smallvec::SmallVec;
use std::sync::{Arc, Mutex};

#[derive(Clone, Debug)]
pub struct PathingGrid {
    pub grid: BoolGrid,
    pub components: UnionFind<usize>,
    pub components_dirty: bool,
    pub allow_diagonal_move: bool,
    pub(crate) context: Arc<Mutex<DefaultSearchContext<Point, i32>>>,
}

impl Default for PathingGrid {
    fn default() -> PathingGrid {
        let grid = PathingGrid {
            grid: BoolGrid::default(),
            components: UnionFind::new(0),
            components_dirty: false,
            allow_diagonal_move: true,
            context: Arc::new(Mutex::new(SearchContext::new())),
        };
        grid
    }
}
impl PathingGrid {
    pub fn neighborhood_points(&self, point: &Point) -> SmallVec<[Point; 8]> {
        if self.allow_diagonal_move {
            point.moore_neighborhood_smallvec()
        } else {
            point.neumann_neighborhood_smallvec()
        }
    }
    pub fn neighborhood_points_and_cost(
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
    pub fn can_move_to(&self, pos: Point, start: Point) -> bool {
        if ALLOW_CORNER_CUTTING {
            self.can_move_to_simple(pos)
        } else {
            debug_assert!((start.x - pos.x).abs() <= 1 && (start.y - pos.y).abs() <= 1);
            self.can_move_to_simple(pos)
                && (!self.grid.get_point(Point::new(start.x, pos.y))
                    && !self.grid.get_point(Point::new(pos.x, start.y)))
        }
    }
    pub fn can_move_to_simple(&self, pos: Point) -> bool {
        self.point_in_bounds(pos) && !self.grid.get_point(pos)
    }
    fn in_bounds(&self, x: i32, y: i32) -> bool {
        self.grid.index_in_bounds(x, y)
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
impl fmt::Display for PathingGrid {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Grid:")?;
        for y in 0..self.grid.height as i32 {
            let values = (0..self.grid.width as i32)
                .map(|x| self.grid.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        Ok(())
    }
}

impl ValueGrid<bool> for PathingGrid {
    fn new(width: usize, height: usize, default_value: bool) -> Self {
        let base_grid = PathingGrid {
            grid: BoolGrid::new(width, height, default_value),

            components: UnionFind::new(width * height),
            components_dirty: false,
            allow_diagonal_move: true,
            context: Arc::new(Mutex::new(SearchContext::new())),
        };
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
        let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
        pathing_grid.allow_diagonal_move = false;
        pathing_grid.set(1, 1, true);
        pathing_grid.generate_components();
        let start = Point::new(0, 0);
        let end = Point::new(2, 2);
        assert!(pathing_grid.reachable(&start, &end));
    }
    // Tests whether allowing diagonals has the expected effect on diagonal reachability in a minimal setting.
    #[test]
    fn test_diagonal_switch_reachable() {
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
        if ALLOW_CORNER_CUTTING {
            assert!(pathing_grid_diag.reachable(&start, &end));
        } else {
            assert!(pathing_grid_diag.unreachable(&start, &end));
        }
    }
}
