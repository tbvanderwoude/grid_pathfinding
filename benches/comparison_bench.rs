use criterion::{black_box, criterion_group, criterion_main, Criterion};
use grid_pathfinding::PathingGrid;
use grid_pathfinding_benchmark::*;
use grid_util::grid::Grid;
use grid_util::point::Point;
use rand::prelude::*;

fn random_grid(
    n: usize,
    rng: &mut StdRng,
    allow_diag: bool,
    pruning: bool,
    fill_rate: f64,
) -> PathingGrid {
    let mut pathing_grid: PathingGrid = PathingGrid::new(n, n, false);
    pathing_grid.allow_diagonal_move = allow_diag;
    pathing_grid.improved_pruning = pruning;
    for x in 0..pathing_grid.width() {
        for y in 0..pathing_grid.height() {
            pathing_grid.set(x, y, rng.gen_bool(fill_rate))
        }
    }
    pathing_grid.generate_components();
    pathing_grid
}
fn random_grid_point(grid: &PathingGrid, rng: &mut StdRng) -> Point {
    Point::new(
        rng.gen_range(0..grid.width()) as i32,
        rng.gen_range(0..grid.height()) as i32,
    )
}

fn test(pathing_grid: &PathingGrid, start: Point, end: Point) -> Option<Vec<Point>> {
    black_box(pathing_grid.get_path_single_goal(start, end, false))
}

fn dao_bench(c: &mut Criterion) {
    let allow_diag = true;
    let pruning = false;
    for name in ["dao/arena", "dao/den312d", "dao/arena2"] {
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
                    test(&pathing_grid, *start, *end);
                }
            })
        });
    }
}

fn criterion_benchmark(c: &mut Criterion) {
    for (allow_diag, pruning) in [(false, false), (true, false), (true, true)] {
        const N: usize = 64;
        const N_GRIDS: usize = 1000;
        const N_PAIRS: usize = 1000;
        let mut rng = StdRng::seed_from_u64(0);
        let mut random_grids: Vec<PathingGrid> = Vec::new();
        for _ in 0..N_GRIDS {
            random_grids.push(random_grid(N, &mut rng, allow_diag, pruning, 0.4))
        }

        let start = Point::new(0, 0);
        let end = Point::new(N as i32 - 1, N as i32 - 1);
        let diag_str = if allow_diag { "8-grid" } else { "4-grid" };
        let improved_str = if pruning { " (improved pruning)" } else { "" };
        c.bench_function(
            format!("1000 random 64x64 {diag_str}s{improved_str}").as_str(),
            |b| {
                b.iter(|| {
                    for grid in &random_grids {
                        test(grid, start, end);
                    }
                })
            },
        );
        let grid = &random_grids[0];
        let mut random_pairs: Vec<(Point, Point)> = Vec::new();
        for _ in 0..N_PAIRS {
            random_pairs.push((
                random_grid_point(&grid, &mut rng),
                random_grid_point(&grid, &mut rng),
            ))
        }
        c.bench_function(
            format!("1000 random start goal pairs on a 64x64 {diag_str}{improved_str}").as_str(),
            |b| {
                b.iter(|| {
                    for (start, end) in &random_pairs {
                        test(&grid, start.clone(), end.clone());
                    }
                })
            },
        );
    }
}

criterion_group!(benches, dao_bench);
criterion_main!(benches);
