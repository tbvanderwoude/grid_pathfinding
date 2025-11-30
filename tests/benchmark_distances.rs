use grid_pathfinding::pathing_grid::PathingGrid;
use grid_pathfinding::solver::{astar::AstarSolver, jps::JPSSolver, GridSolver};
use grid_pathfinding_benchmark::get_benchmark;
use grid_util::*;
use std::fs::File;
use std::io::{BufWriter, Write};

fn save_path(path: Vec<Point>, filename: &str) -> std::io::Result<()> {
    let f = File::create(filename)?;
    let mut w = BufWriter::new(f);
    for p in path {
        writeln!(w, "{},{}", p.x, p.y)?;
    }
    Ok(())
}

#[test]
fn verify_solution_distance_jps() {
    let bench_set = ["dao/arena", "dao/lak107d", "dao/den101d"];
    for name in bench_set {
        let (bool_grid, scenarios) = get_benchmark(name.to_owned());
        let mut pathing_grid: PathingGrid<true> =
            PathingGrid::new(bool_grid.width, bool_grid.height, true);

        pathing_grid.grid = bool_grid.clone();
        pathing_grid.generate_components();
        let mut solver = JPSSolver::new(&pathing_grid, false);
        solver.initialize(&pathing_grid);

        for (start, end, distance) in &scenarios {
            println!("Start: {start}; End: {end}; Distance: {distance}");
            let path = solver
                .get_path_single_goal(&mut pathing_grid, *start, *end)
                .unwrap();
            save_path(path.clone(), "path.csv").unwrap();
            let float_cost = solver.get_path_cost_float(&path, &pathing_grid);
            println!("My distance: {float_cost}");
            if *distance >= 0.01 {
                let delta_dist = (float_cost - distance).abs() / distance;
                assert!(delta_dist < 0.05);
            }
        }
    }
}

#[test]
fn verify_solution_distance_astar() {
    let bench_set = ["dao/arena", "dao/lak107d", "dao/den101d"];
    let solver = AstarSolver::new();

    for name in bench_set {
        let (bool_grid, scenarios) = get_benchmark(name.to_owned());
        let mut pathing_grid: PathingGrid<true> =
            PathingGrid::new(bool_grid.width, bool_grid.height, true);
        pathing_grid.grid = bool_grid.clone();
        pathing_grid.generate_components();
        for (start, end, distance) in &scenarios {
            println!("Start: {start}; End: {end}; Distance: {distance}");
            let path = solver
                .get_path_single_goal(&mut pathing_grid, *start, *end)
                .unwrap();
            save_path(path.clone(), "path.csv").unwrap();
            let float_cost = solver.get_path_cost_float(&path, &pathing_grid);
            println!("My distance: {float_cost}");
            if *distance >= 0.01 {
                let delta_dist = (float_cost - distance).abs() / distance;
                assert!(delta_dist < 0.05);
            }
        }
    }
}
