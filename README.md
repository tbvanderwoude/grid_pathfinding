# grid_pathfinding

A grid-based pathfinding system. Implements [Jump Point Search](https://en.wikipedia.org/wiki/Jump_point_search) with 
[improved pruning rules](https://www.researchgate.net/publication/287338108_Improving_jump_point_search) for speedy pathfinding. Pre-computes
[connected components](https://en.wikipedia.org/wiki/Component_(graph_theory))
to avoid flood-filling behaviour if no path exists. Both [4-neighborhood](https://en.wikipedia.org/wiki/Von_Neumann_neighborhood) and [8-neighborhood](https://en.wikipedia.org/wiki/Moore_neighborhood) grids are supported and a custom variant of JPS is implemented for the 4-neighborhood. 

### Example
Below a [simple 8-grid example](examples/simple_8.rs) is given which illustrates how to set a basic problem and find a path.
```rust,no_run
use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;

// In this example a path is found on a 3x3 grid with shape
//  ___
// |S  |
// | # |
// |  E|
//  ___
// where
// - # marks an obstacle
// - S marks the start
// - E marks the end

fn main() {
    let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
    pathing_grid.set(1, 1, true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(0, 0);
    let end = Point::new(2, 2);
    let path = pathing_grid
        .get_path_single_goal(start, end, false)
        .unwrap();
    println!("Path:");
    for p in path {
        println!("{:?}", p);
    }
}
```
This assumes an 8-neighborhood, which is the default grid type. The same problem can be solved for a 4-neighborhood, disallowing diagonal moves, by adding the line
```rust,no_run
pathing_grid.allow_diagonal_move = false;
```
prior to component generation, which is done in example [simple_4](examples/simple_4.rs).



See [examples](examples/) for finding paths with multiple goals and generating waypoints instead of full paths.

### Goal of crate
The long-term goal of this crate is to provide a fast pathfinding implementation for grids as well as support
for features like multi-tile pathfinding and [multi-agent pathfinding](https://en.wikipedia.org/wiki/Multi-agent_pathfinding).
