use grid_pathfinding::{waypoints_to_path, PathingGrid};
use grid_util::grid::Grid;
use grid_util::point::Point;

// This example illustrates the difference between waypoints and paths.
// A path is found on a 5x5 grid with shape
// -----
// |S    |
// | #   |
// |     |
// |     |
// |    E|
//  -----
// where
// - S marks the start
// - E marks the end
// First the waypoints are found using [get_waypoints_single_goal](PathingGrid::get_waypoints_single_goal).
// These are then expanded using [get_waypoints_single_goal](PathingGrid::get_waypoints_single_goal).
// Lastly, [get_path_single_goal](PathingGrid::get_path_single_goal) is used to directly get the
// path, as a shorthand for the two previous calls.

fn main() {
    let mut pathing_grid: PathingGrid = PathingGrid::new(5, 5, false);
    pathing_grid.set(1, 1, true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(0, 0);
    let end = Point::new(4, 4);
    if let Some(path) = pathing_grid.get_waypoints_single_goal(start, end, false) {
        println!("Waypoints:");
        for p in &path {
            println!("{:?}", p);
        }
        println!("\nPath generated from waypoints:");
        for p in waypoints_to_path(path) {
            println!("{:?}", p);
        }
    }
    println!("\nDirectly computed path");
    let expanded_path = pathing_grid
        .get_path_single_goal(start, end, false)
        .unwrap();
    for p in expanded_path {
        println!("{:?}", p);
    }
}
