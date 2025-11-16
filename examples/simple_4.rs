use grid_pathfinding::Pathfinder;
use grid_util::grid::ValueGrid;
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
//
// Nodes have a 4-neighborhood

fn main() {
    let mut pathing_grid: Pathfinder<false> = Pathfinder::new(3, 3, false);
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
