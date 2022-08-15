use grid_pathfinding::{waypoints_to_path, PathingGrid};
use grid_util::grid::Grid;
use grid_util::point::Point;

/// This example illustrates the type of output [get_path_single_goal](Self::get_path_single_goal)
/// gives. It is a minimal representation that can be expanded using [expand_path]
/// A path is found on a 5x5 grid with shape
/// S
///  #
///
///
///     E
/// where
/// - S marks the start
/// - E marks the end
fn main() {
    let mut pathing_grid: PathingGrid = PathingGrid::new(5, 5, false);
    pathing_grid.set(1, 1, true);
    // pathing_grid.set_rectangle(&Rect::new(1,1,3,3),true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(0, 0);
    let end = Point::new(4, 4);
    if let Some(path) = pathing_grid.get_waypoints_single_goal(start, end, false) {
        println!("Unexpanded path:");
        for p in &path {
            println!("{:?}", p);
        }
        println!("Expanded path:");
        for p in waypoints_to_path(path) {
            println!("{:?}", p);
        }
    }
    /// There also exists a variant of [PathingGrid::get_path_single_goal],
    /// [PathingGrid::get_expanded_path_single_goal], which expands the path automatically.
    let expanded_path = pathing_grid.get_path_single_goal(start, end, false).unwrap();
    println!("{:?}",expanded_path);
}
