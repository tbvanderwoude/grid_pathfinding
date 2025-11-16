use criterion::{criterion_group, criterion_main, Criterion};
use grid_pathfinding::{
    pathing_grid::PathingGrid,
    solver::{astar::AstarSolver, jps::JPSSolver, GridSolver},
    Pathfinder,
};
use grid_pathfinding_benchmark::*;
use grid_util::grid::ValueGrid;
use std::hint::black_box;

fn dao_bench(c: &mut Criterion) {
    for (allow_diag, pruning) in [(true, false)] {
        let bench_set = if allow_diag {
            ["dao/arena", "dao/den312d", "dao/arena2"]
        } else {
            ["dao/arena", "dao/den009d", "dao/den312d"]
        };
        for name in bench_set {
            let (bool_grid, scenarios) = get_benchmark(name.to_owned());
            let mut pathing_grid: Pathfinder =
                Pathfinder::new(bool_grid.width, bool_grid.height, true);
            pathing_grid.grid = bool_grid.clone();
            pathing_grid.allow_diagonal_move = allow_diag;
            pathing_grid.improved_pruning = pruning;
            pathing_grid.initialize();
            pathing_grid.generate_components();
            let diag_str = if allow_diag { "8-grid" } else { "4-grid" };
            let improved_str = if pruning { " (improved pruning)" } else { "" };

            c.bench_function(format!("{name}, {diag_str}{improved_str}").as_str(), |b| {
                b.iter(|| {
                    for (start, end, _) in &scenarios {
                        black_box(pathing_grid.get_path_single_goal(*start, *end, false));
                    }
                })
            });
        }
    }
}

fn dao_bench_jps<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    for (pruning) in [false] {
        let bench_set = if ALLOW_DIAGONAL {
            ["dao/arena", "dao/den312d", "dao/arena2"]
        } else {
            ["dao/arena", "dao/den009d", "dao/den312d"]
        };
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

            c.bench_function(
                format!("{name}, JPS {diag_str}{improved_str}").as_str(),
                |b| {
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
                },
            );
        }
    }
}
fn dao_bench_astar<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    let bench_set = if ALLOW_DIAGONAL {
        ["dao/arena", "dao/den312d", "dao/arena2"]
    } else {
        ["dao/arena", "dao/den009d", "dao/den312d"]
    };
    for name in bench_set {
        let (bool_grid, scenarios) = get_benchmark(name.to_owned());
        let mut pathing_grid: PathingGrid<ALLOW_DIAGONAL> =
            PathingGrid::new(bool_grid.width, bool_grid.height, true);
        pathing_grid.grid = bool_grid.clone();
        pathing_grid.generate_components();
        let solver = AstarSolver::new();
        let diag_str = if ALLOW_DIAGONAL { "8-grid" } else { "4-grid" };

        c.bench_function(format!("{name}, A* {diag_str}").as_str(), |b| {
            b.iter(|| {
                for (start, end, _) in &scenarios {
                    black_box(solver.get_path_single_goal(&mut pathing_grid, *start, *end, false));
                }
            })
        });
    }
}
criterion_group!(
    benches,
    dao_bench,
    dao_bench_jps<false>,
    dao_bench_jps<true>,
    dao_bench_astar<false>,
    dao_bench_astar<true>
);
criterion_main!(benches);
