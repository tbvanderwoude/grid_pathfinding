use grid_pathfinding::*;
use grid_pathfinding_benchmark::get_benchmark;
use grid_util::*;

#[test]
fn verify_solution_distance() {
    for pruning in [false, true] {
        let bench_set = ["dao/arena", "dao/arena2"];
        for name in bench_set {
            let (bool_grid, scenarios) = get_benchmark(name.to_owned());
            let mut pathing_grid: PathingGrid =
                PathingGrid::new(bool_grid.width, bool_grid.height, true);
            pathing_grid.grid = bool_grid.clone();
            pathing_grid.allow_diagonal_move = true;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.initialize();
            pathing_grid.generate_components();
            for (start, end, distance) in &scenarios {
                println!("Distance: {distance}");
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
                let float_cost = convert_cost_to_unit_cost_float(total_cost_int);
                println!("My distance: {float_cost}");
                let delta_dist = (float_cost - distance).abs() / distance;
                assert!(delta_dist < 0.01);
            }
        }
    }
}
