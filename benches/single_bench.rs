use criterion::{criterion_group, criterion_main, Criterion};
use grid_pathfinding::{
    pathing_grid::PathingGrid,
    solver::{jps::JPSSolver, GridSolver},
};
use grid_pathfinding_benchmark::*;
use grid_util::grid::ValueGrid;
use std::hint::black_box;

fn dao_bench_single<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    for pruning in [false] {
        let bench_set = ["dao/arena"];
        for name in bench_set {
            let (bool_grid, scenarios) = get_benchmark(name.to_owned());
            let mut pathing_grid: PathingGrid<ALLOW_DIAGONAL> =
                PathingGrid::new(bool_grid.width, bool_grid.height, true);
            pathing_grid.grid = bool_grid.clone();
            pathing_grid.generate_components();
            let mut solver = JPSSolver::new(&pathing_grid, pruning);
            solver.initialize(&pathing_grid);
            let diag_str = if ALLOW_DIAGONAL { "8-grid" } else { "4-grid" };
            let improved_str = if pruning { " (improved pruning)" } else { "" };

            c.bench_function(format!("{name}, {diag_str}{improved_str}").as_str(), |b| {
                b.iter(|| {
                    for (start, end, _) in &scenarios {
                        black_box(solver.get_path_single_goal(
                            &mut pathing_grid,
                            *start,
                            *end,
                            false,
                        ));
                    }
                })
            });
        }
    }
}

criterion_group!(benches, dao_bench_single<false>, dao_bench_single<true>);
criterion_main!(benches);
