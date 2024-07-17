use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;
use grid_util::Rect;

// The heuristic_factor can be set to scale the heuristic, causing nodes that are closer to the goal (ignoring obstacles)
// to be evaluated quicker than in normal operation. This is called Weighted A* and it can speed up the algorithm in certain scenarios.

fn main() {
    const N: i32 = 30;
    let mut pathing_grid: PathingGrid = PathingGrid::new(N as usize, N as usize, true);
    pathing_grid.heuristic_factor = 1.3;
    pathing_grid.set_rectangle(&Rect::new(1, 1, N - 2, N - 2), false);
    pathing_grid.set_rectangle(&Rect::new(8, 8, 8, 8), true);
    pathing_grid.set_rectangle(&Rect::new(0, 3, 6, 6), true);
    pathing_grid.set_rectangle(&Rect::new(10, 0, 6, 6), true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(1, 1);
    let end = Point::new(N - 3, N - 3);
    let path = pathing_grid
        .get_path_single_goal(start, end, false)
        .unwrap();
    println!("Path:");
    for p in path {
        println!("{:?}", p);
    }
}
