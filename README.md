# grid_pathfinding

A grid-based pathfinding system. Implements [Jump Point Search](https://en.wikipedia.org/wiki/Jump_point_search) with 
[improved pruning rules](https://www.researchgate.net/publication/287338108_Improving_jump_point_search) for speedy pathfinding. Pre-computes
[connected components](https://en.wikipedia.org/wiki/Component_(graph_theory))
to avoid flood-filling behavior if no path exists. Both [4-neighborhood](https://en.wikipedia.org/wiki/Von_Neumann_neighborhood) and [8-neighborhood](https://en.wikipedia.org/wiki/Moore_neighborhood) grids are supported and a custom variant of JPS is implemented for the 4-neighborhood. 

### Example
Below a [simple 8-grid example](examples/simple_8.rs) is given, illustrating how to set a basic problem and find a path.
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
before component generation, which is done in example [simple_4](examples/simple_4.rs).



See [examples](examples/) for finding paths with multiple goals and generating waypoints instead of full paths.

### Benchmarks
The system can be benchmarked using scenarios from the [Moving AI 2D pathfinding benchmarks](https://movingai.com/benchmarks/grids.html). The [grid_pathfinding_benchmark](grid_pathfinding_benchmark) utility crate provides general support for loading these files. The default benchmark executed using `cargo bench` runs three scenario sets from the [Dragon Age: Origins](https://movingai.com/benchmarks/dao/index.html): `dao/arena`, `dao/den312` and `dao/arena2`. Running these requires the corresponding map and scenario files to be saved in folders called `maps/dao` and `scenarios/dao`.

A baseline can be set using
```bash
cargo bench -- --save-baseline main
```
New runs can be compared to this baseline using 
```bash
cargo bench -- --baseline main
```

### Performance
Using an i7-11700K octa-core running at 4.8 GHz, running the `dao/arena2` set of 910 scenarios on a 281x209 grid takes 63.2 ms using JPS allowing diagonals and with improved pruning disabled. Using default neighbor generation as in normal A* (enabled by setting `GRAPH_PRUNING = false`) makes this take 702 ms, more than a factor 10 difference. As a rule, the relative difference increases as maps get larger, with the `dao/arena` set of 130 scenarios on a 49x49 grid taking 360 us and 526 us respectively with and without pruning. 

An existing C++ [JPS implementation](https://github.com/nathansttt/hog2) runs the same `dao/arena2` scenarios in roughly 33 ms. The fastest solver known to the author is the [l1-path-finder](https://mikolalysenko.github.io/l1-path-finder/www/) (implemented in Javascript) which can do this in 21 ms using A* with landmarks (for a 4-neighborhood).

### Goal of crate
The long-term goal of this crate is to provide a fast off-the-shelf pathfinding implementation for uniform-cost grids.