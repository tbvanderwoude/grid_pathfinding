use grid_util::grid::Grid;
use grid_util::point::Point;
use grid_util::rect::Rect;
use grid_pathfinding::PathingGrid;

// In this example a path is found on a grid with shape
// #####
// #S  #
// # # #
// #  E#
// #####
// S marks the start
// E marks the end
fn main() {
    let mut pathing_grid: PathingGrid = PathingGrid::new(5,5,true);
    pathing_grid.set_rectangle(&Rect::new(1,1,3,3,),false);
    pathing_grid.set(2,2,true);
    pathing_grid.generate_components();
    let start = Point::new(1,1);
    let end = Point::new(3,3);
    if let Some(path) = pathing_grid.get_path_single_goal(start,end,false){
        println!("A path has been found:");
        for p in path{
            println!("{:?}",p);
        }
    }
}