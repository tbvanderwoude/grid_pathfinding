use criterion::{criterion_group, criterion_main, Criterion};
use grid_pathfinding::{
    pathing_grid::PathingGrid,
    solver::{
        alt::ALTSolver, astar::AstarSolver, dijkstra::DijkstraSolver, jps::JPSSolver, GridSolver,
    },
    Pathfinder,
};
use grid_pathfinding_benchmark::*;
use grid_util::{grid::ValueGrid, Point};
use rand::{rngs::StdRng, Rng, SeedableRng};
use smallvec::{smallvec, SmallVec};
use std::hint::black_box;

fn dao_bench<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    let arr: SmallVec<[bool; 2]> = if ALLOW_DIAGONAL {
        smallvec![false]
    } else {
        smallvec![false]
    };
    let bench_set = if ALLOW_DIAGONAL {
        ["dao/arena2"]
    } else {
        ["dao/arena2"]
    };
    for pruning in arr {
        for name in bench_set {
            let (bool_grid, scenarios) = get_benchmark(name.to_owned());
            let mut pathing_grid: Pathfinder<ALLOW_DIAGONAL> =
                Pathfinder::new(bool_grid.width, bool_grid.height, true);
            pathing_grid.grid = bool_grid.clone();
            pathing_grid.improved_pruning = pruning;
            pathing_grid.initialize();
            pathing_grid.generate_components();
            let diag_str = if ALLOW_DIAGONAL { "8-grid" } else { "4-grid" };
            let improved_str = if pruning { " (improved pruning)" } else { "" };

            c.bench_function(format!("{name}, {diag_str}{improved_str}").as_str(), |b| {
                b.iter(|| {
                    for (start, end, _) in &scenarios {
                        black_box(pathing_grid.get_path_single_goal(*start, *end));
                    }
                })
            });
        }
    }
}

fn dao_bench_solver<const ALLOW_DIAGONAL: bool, S, FS>(
    c: &mut Criterion,
    solver_name: &str,
    create_solver: FS,
) where
    S: GridSolver,
    FS: Fn(&mut PathingGrid<ALLOW_DIAGONAL>) -> S,
{
    let bench_set = if ALLOW_DIAGONAL {
        ["dao/arena2"]
    } else {
        ["dao/arena2"]
    };
    for name in bench_set {
        let (bool_grid, scenarios) = get_benchmark(name.to_owned());
        let mut pathing_grid: PathingGrid<ALLOW_DIAGONAL> =
            PathingGrid::new(bool_grid.width, bool_grid.height, true);
        pathing_grid.grid = bool_grid.clone();
        pathing_grid.generate_components();
        let solver = create_solver(&mut pathing_grid);
        let diag_str = if ALLOW_DIAGONAL { "8-grid" } else { "4-grid" };

        c.bench_function(format!("{name}, {solver_name} {diag_str}").as_str(), |b| {
            b.iter(|| {
                for (start, end, _) in &scenarios {
                    black_box(solver.get_path_single_goal(&mut pathing_grid, *start, *end));
                }
            })
        });
    }
}
pub fn generate_landmarks_mc<const ALLOW_DIAGONAL: bool>(
    pathing_grid: &PathingGrid<ALLOW_DIAGONAL>,
    number: usize,
) -> Vec<Point> {
    let mut landmarks = Vec::new();
    let mut rng = StdRng::seed_from_u64(0);
    while landmarks.len() < number {
        let p = Point::new(
            rng.random_range(0..pathing_grid.width() as i32),
            rng.random_range(0..pathing_grid.height() as i32),
        );
        if pathing_grid.can_move_to_simple(p) {
            landmarks.push(p);
        }
    }
    landmarks
}

fn dao_bench_jps<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    let arr: SmallVec<[bool; 2]> = if ALLOW_DIAGONAL {
        smallvec![false]
    } else {
        smallvec![false]
    };
    for pruning in arr {
        let improved_str = if pruning { " (improved pruning)" } else { "" };
        dao_bench_solver(
            c,
            format!("JPS{improved_str}").as_str(),
            |pathing_grid: &mut PathingGrid<ALLOW_DIAGONAL>| {
                let mut solver = JPSSolver::new(&pathing_grid, pruning);
                solver.initialize(&pathing_grid);
                solver
            },
        );
    }
}

const N_LANDMARKS: usize = 64;
fn dao_bench_alt_mc<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    dao_bench_solver(
        c,
        "ALT (MC)",
        |pathing_grid: &mut PathingGrid<ALLOW_DIAGONAL>| {
            let landmarks = generate_landmarks_mc(&pathing_grid, N_LANDMARKS);
            ALTSolver::new_from_landmarks(landmarks, pathing_grid)
        },
    );
}
fn dao_bench_alt_greedy<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    dao_bench_solver(
        c,
        "ALT (greedy)",
        |pathing_grid: &mut PathingGrid<ALLOW_DIAGONAL>| {
            let landmarks = generate_landmarks_mc(&pathing_grid, 1);
            ALTSolver::new_greedy(landmarks[0], N_LANDMARKS, pathing_grid)
        },
    );
}
fn dao_bench_astar<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    dao_bench_solver(c, "Astar", |_: &mut PathingGrid<ALLOW_DIAGONAL>| {
        AstarSolver::new()
    });
}

fn dao_bench_dijkstra<const ALLOW_DIAGONAL: bool>(c: &mut Criterion) {
    dao_bench_solver(c, "Dijkstra", |_: &mut PathingGrid<ALLOW_DIAGONAL>| {
        DijkstraSolver
    });
}

criterion_group!(
    benches,
    dao_bench<true>,
    dao_bench_jps<true>,
    dao_bench_alt_greedy<true>,
    dao_bench_alt_mc<true>,
);
criterion_main!(benches);
