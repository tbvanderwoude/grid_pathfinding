use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;
use grid_util::Rect;

fn main() {
    let mut pathing_grid: PathingGrid = PathingGrid::new(10, 10, false);
    pathing_grid.set_rectangle(&Rect::new(1,1,2,2), true);
    pathing_grid.set_rectangle(&Rect::new(5,0,2,2), true);
    pathing_grid.set_rectangle(&Rect::new(0,5,2,2), true);
    pathing_grid.set_rectangle(&Rect::new(8,8,2,2), true);
    pathing_grid.allow_diagonal_move = false;
    pathing_grid.improved_pruning = false;
    pathing_grid.generate_components();
    let start = Point::new(0, 0);
    let end = Point::new(7, 7);
    let waypoints = pathing_grid
        .get_waypoints_single_goal(start, end, false)
        .unwrap();
    println!("{:?}",waypoints);
}
