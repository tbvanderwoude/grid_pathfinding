use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;

/// In this example a path is found on a 3x3 grid with shape
/// S
///  #
///   E
/// where
/// - S marks the start
/// - E marks the end
/// This is similar to the simple example but here the borders of the grid are implicit.
fn main() {
    let mut pathing_grid: PathingGrid = PathingGrid::new(3, 3, false);
    pathing_grid.set(1, 1, true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(0, 0);
    let end = Point::new(2, 2);
    if let Some(path) = pathing_grid.get_waypoints_single_goal(start, end, false) {
        println!("A path has been found:");
        for p in path {
            println!("{:?}", p);
        }
    }
}
