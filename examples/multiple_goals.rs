use grid_pathfinding::Pathfinder;
use grid_util::grid::ValueGrid;
use grid_util::point::Point;

// In this example a path is found to one of two goals on a 3x3 grid with shape
//  ___
// |S G|
// | # |
// |  G|
//  ___
// where
// - \# marks an obstacle
// - S marks the start
// - G marks a goal
// The found path moves to the closest goal, which is the top one.

fn main() {
    let mut pathing_grid: Pathfinder<true> = Pathfinder::new(3, 3, false);
    pathing_grid.set(1, 1, true);
    pathing_grid.generate_components();
    println!("{}", pathing_grid);
    let start = Point::new(0, 0);
    let goal_1 = Point::new(2, 0);
    let goal_2 = Point::new(2, 2);
    let goals = vec![&goal_1, &goal_2];
    let (selected_goal, path) = pathing_grid.get_path_multiple_goals(start, goals).unwrap();
    println!("Selected goal: {:?}\n", selected_goal);
    println!("Path:");
    for p in path {
        println!("{:?}", p);
    }
}
