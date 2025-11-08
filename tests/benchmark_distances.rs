use grid_pathfinding::*;
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
fn verify_solution_distance() {
    let bench_set = ["dao/arena", "dao/lak107d", "dao/den101d"];
    for name in bench_set {
        let (bool_grid, scenarios) = get_benchmark(name.to_owned());
        let mut pathing_grid: PathingGrid =
            PathingGrid::new(bool_grid.width, bool_grid.height, true);
        pathing_grid.grid = bool_grid.clone();
        pathing_grid.allow_diagonal_move = true;
        pathing_grid.improved_pruning = false;
        pathing_grid.initialize();
        pathing_grid.generate_components();
        for (start, end, distance) in &scenarios {
            println!("Start: {start}; End: {end}; Distance: {distance}");
            let path = pathing_grid
                .get_path_single_goal(*start, *end, false)
                .unwrap();
            let mut v = path[0];
            let n = path.len();
            let mut total_cost_int = 0;
            for i in 1..n {
                let v_old = v;
                v = path[i];
                let cost = pathing_grid.heuristic(&v_old, &v);
                total_cost_int += cost;
            }
            save_path(path, "path.csv").unwrap();
            let float_cost = convert_cost_to_unit_cost_float(total_cost_int);
            println!("My distance: {float_cost}");
            if *distance >=0.01{
                let delta_dist = (float_cost - distance).abs() / distance;
                assert!(delta_dist < 0.05);
            }
        }
    }
}
