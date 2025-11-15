use core::fmt;
use grid_util::{Direction, Point, SimpleValueGrid, ValueGrid};
use smallvec::SmallVec;

use crate::{
    pathing_grid::PathingGrid, solver::GridSolver, ALLOW_CORNER_CUTTING, C, D, E, N_SMALLVEC_SIZE,
};

#[derive(Clone, Debug)]
pub struct JPSSolver {
    pub jump_point: SimpleValueGrid<u8>,
    pub neighbours: SimpleValueGrid<u8>,
    pub improved_pruning: bool,
}

impl GridSolver for JPSSolver {
    type Successors = SmallVec<[(Point, i32); N_SMALLVEC_SIZE]>;

    fn successors<F>(
        &self,
        grid: &PathingGrid,
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
                for (n, c) in self.pruned_neighborhood(dir, node, grid) {
                    let dir = node.dir_obj(&n);
                    // Jumps the neighbor, skipping over unnecessary nodes.
                    if let Some((jumped_node, cost)) = self.jump(*node, c, dir, goal, grid) {
                        // If improved pruning is enabled, expand any diagonal unforced nodes
                        if self.improved_pruning
                            && dir.diagonal()
                            && !goal(&jumped_node)
                            && !self.is_forced(dir, &jumped_node)
                        {
                            // Recursively expand the unforced diagonal node
                            let jump_points = self.successors(grid, parent, &jumped_node, goal);

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
                grid.neighborhood_points_and_cost(node)
            }
        }
    }

    /// Uses C as cost for cardinal (straight) moves and D for diagonal moves.
    fn heuristic(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32 {
        if grid.allow_diagonal_move {
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

    /// Uses C as cost for cardinal (straight) moves and D for diagonal moves.
    fn cost(&self, grid: &PathingGrid, p1: &Point, p2: &Point) -> i32 {
        self.heuristic(grid, p1, p2)
    }
}
impl JPSSolver {
    pub fn new(grid: &PathingGrid, improved_pruning: bool) -> JPSSolver {
        let mut solver = JPSSolver {
            jump_point: SimpleValueGrid::new(grid.width(), grid.height(), 0),
            neighbours: SimpleValueGrid::new(grid.width(), grid.height(), 0b11111111),
            improved_pruning,
        };
        solver.initialize(grid);
        solver
    }

    fn is_forced(&self, dir: Direction, node: &Point) -> bool {
        let dir_num = dir.num();
        self.jump_point.get_point(*node) & (1 << dir_num) != 0
    }
    /// The neighbour indexing used here corresponds to that used in [grid_util::Direction].
    pub fn indexed_neighbor(&self, node: &Point, index: i32) -> bool {
        (self.neighbours.get_point(*node) & 1 << (index.rem_euclid(8))) != 0
    }
    fn width(&self) -> usize {
        self.neighbours.width()
    }
    fn height(&self) -> usize {
        self.neighbours.height()
    }
    fn in_bounds(&self, x: i32, y: i32) -> bool {
        self.neighbours.index_in_bounds(x, y)
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
    fn forced_mask(&self, node: &Point) -> u8 {
        let mut forced_mask: u8 = 0;
        for dir_num in 0..8 {
            if dir_num % 2 == 1 {
                if ALLOW_CORNER_CUTTING && !self.indexed_neighbor(node, 3 + dir_num)
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
        grid: &PathingGrid,
    ) -> impl Iterator<Item = (Point, i32)> + 'a {
        let dir_num = dir.num();
        let mut n_mask: u8;
        let mut neighbours = self.neighbours.get_point(*node);
        if !grid.allow_diagonal_move {
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
            if ALLOW_CORNER_CUTTING {
                n_mask = 0b00000001 << dir_num;
                if !self.indexed_neighbor(node, 2 + dir_num) {
                    n_mask |= 1 << ((dir_num + 1) % 8);
                }
                if !self.indexed_neighbor(node, 6 + dir_num) {
                    n_mask |= 1 << ((dir_num + 7) % 8);
                }
            } else {
                // TODO: look into whether this is minimal, this at least makes the algorithm
                // optimal and complete following the no corner cutting rule
                n_mask = 0b11010111_u8.rotate_left(dir_num as u32);
            }
        }
        let comb_mask = neighbours & n_mask;
        (0..8)
            .step_by(if grid.allow_diagonal_move { 1 } else { 2 })
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
        grid: &PathingGrid,
    ) -> Option<(Point, i32)>
    where
        F: Fn(&Point) -> bool,
    {
        debug_assert!(!direction.diagonal());
        loop {
            initial = initial + direction;
            if !grid.can_move_to_simple(initial) {
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
        grid: &PathingGrid,
    ) -> Option<(Point, i32)>
    where
        F: Fn(&Point) -> bool,
    {
        let mut new_initial: Point;
        loop {
            new_initial = initial + direction;
            if !grid.can_move_to(new_initial, initial) {
                return None;
            }
            initial = new_initial;

            if goal(&initial) || self.is_forced(direction, &initial) {
                return Some((initial, cost));
            }
            if direction.diagonal()
                && (self
                    .jump_straight(initial, 1, direction.x_dir(), goal, grid)
                    .is_some()
                    || self
                        .jump_straight(initial, 1, direction.y_dir(), goal, grid)
                        .is_some())
            {
                return Some((initial, cost));
            }

            // When using a 4-neighborhood (specified by setting allow_diagonal_move to false),
            // jumps perpendicular to the direction are performed. This is necessary to not miss the
            // goal when passing by.
            if !grid.allow_diagonal_move || !ALLOW_CORNER_CUTTING && !direction.diagonal() {
                if grid.allow_diagonal_move {
                    let diag_1 = direction.rotate_ccw(1);
                    let diag_2 = direction.rotate_cw(1);
                    if self.jump(initial, 1, diag_1, goal, grid).is_some()
                        || self.jump(initial, 1, diag_2, goal, grid).is_some()
                    {
                        return Some((initial, cost));
                    }
                }
                let perp_1 = direction.rotate_ccw(2);
                let perp_2 = direction.rotate_cw(2);
                if self.jump_straight(initial, 1, perp_1, goal, grid).is_some()
                    || self.jump_straight(initial, 1, perp_2, goal, grid).is_some()
                {
                    return Some((initial, cost));
                }
            }

            // See comment in pruned_neighborhood about cost calculation
            cost += (direction.num() % 2) * (D - C) + C;
        }
    }

    pub fn set_jumppoints(&mut self, point: Point) {
        let value = self.forced_mask(&point);
        self.jump_point.set_point(point, value);
    }
    pub fn fix_jumppoints(&mut self, point: Point, grid: &PathingGrid) {
        self.set_jumppoints(point);
        for p in grid.neighborhood_points(&point) {
            if grid.point_in_bounds(p) {
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
    /// Updates the neighbours
    fn set(&mut self, x: i32, y: i32, blocked: bool) {
        let p = Point::new(x, y);
        self.update_neighbours(p.x, p.y, blocked);
    }

    pub fn update_all_neighbours(&mut self, grid: &PathingGrid) {
        for x in 0..self.width() as i32 {
            for y in 0..self.height() as i32 {
                self.update_neighbours(x, y, grid.get(x, y));
            }
        }
    }
    pub fn initialize(&mut self, grid: &PathingGrid) {
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
        self.update_all_neighbours(grid);
        self.set_all_jumppoints();
    }
}

impl fmt::Display for JPSSolver {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "Neighbours:")?;
        for y in 0..self.neighbours.height as i32 {
            let values = (0..self.neighbours.width as i32)
                .map(|x| self.neighbours.get(x, y) as i32)
                .collect::<Vec<i32>>();
            writeln!(f, "{:?}", values)?;
        }
        Ok(())
    }
}
