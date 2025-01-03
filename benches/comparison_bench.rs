use criterion::{black_box, criterion_group, criterion_main, Criterion};
use grid_pathfinding::PathingGrid;
use grid_pathfinding_benchmark::*;
use grid_util::grid::Grid;

fn dao_bench(c: &mut Criterion) {
    for (allow_diag, pruning) in [(true, false), (true, true)] {
        let bench_set = if allow_diag {
            ["dao/arena", "dao/den312d", "dao/arena2"]
        } else {
            ["dao/arena", "dao/den009d", "dao/den312d"]
        };
        for name in bench_set {
            let (bool_grid, scenarios) = get_benchmark(name.to_owned());
            let mut pathing_grid: PathingGrid =
                PathingGrid::new(bool_grid.width, bool_grid.height, true);
            pathing_grid.grid = bool_grid.clone();
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.update_all_neighbours();
            pathing_grid.generate_components();
            let diag_str = if allow_diag { "8-grid" } else { "4-grid" };
            let improved_str = if pruning { " (improved pruning)" } else { "" };

            c.bench_function(format!("{name}, {diag_str}{improved_str}").as_str(), |b| {
                b.iter(|| {
                    for (start, end) in &scenarios {
                        black_box(pathing_grid.get_path_single_goal(*start, *end, false));
                    }
                })
            });
        }
    }
}

criterion_group!(benches, dao_bench);
criterion_main!(benches);
