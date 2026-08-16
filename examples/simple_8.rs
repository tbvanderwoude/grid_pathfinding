use grid_pathfinding::Pathfinder;
use grid_util::{grid::ValueGrid, point::Point};

// A path is found on a 3x3 grid with shape
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
// Diagonal moves are allowed, corner cutting can be toggled using CUT_CORNERS

fn main() {
    const CUT_CORNERS: bool = false;
    let mut pathing_grid = Pathfinder::<true, CUT_CORNERS>::new(3, 3, false);
    pathing_grid.set(1, 1, true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(0, 0);
    let end = Point::new(2, 2);
    let path = pathing_grid.get_path_single_goal(start, end).unwrap();
    println!("Path:");
    for p in path {
        println!("{:?}", p);
    }
}
