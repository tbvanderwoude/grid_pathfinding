use grid_pathfinding::Pathfinder;
use grid_pathfinding_benchmark::*;
use grid_util::grid::ValueGrid;
use grid_util::point::Point;
use std::time::{Duration, Instant};

fn main() {
    let benchmark_names = get_benchmark_names();
    let mut total_time = Duration::ZERO;
    for name in benchmark_names {
        println!("Benchmark name: {}", name);

        let (bool_grid, scenarios) = get_benchmark(name);
        // for (allow_diag, pruning) in [(false, false), (true, false), (true, true)] {
        let mut pathing_grid: Pathfinder<true> =
            Pathfinder::new(bool_grid.width, bool_grid.height, true);
        pathing_grid.grid = bool_grid.clone();
        pathing_grid.improved_pruning = false;
        pathing_grid.initialize();
        pathing_grid.generate_components();
        let number_of_scenarios = scenarios.len() as u32;
        let before = Instant::now();
        run_scenarios(&pathing_grid, &scenarios);
        let elapsed = before.elapsed();
        println!(
            "\tElapsed time: {:.2?}; per scenario: {:.2?}",
            elapsed,
            elapsed / number_of_scenarios
        );
        total_time += elapsed;
    }
    println!("\tTotal benchmark time: {:.2?}", total_time);
}

pub fn run_scenarios<const ALLOW_DIAGONAL: bool>(
    pathing_grid: &Pathfinder<ALLOW_DIAGONAL>,
    scenarios: &Vec<(Point, Point, f64)>,
) {
    for (start, goal, _) in scenarios {
        let path: Option<Vec<Point>> = pathing_grid.get_waypoints_single_goal(*start, *goal, false);
        assert!(path.is_some());
    }
}
