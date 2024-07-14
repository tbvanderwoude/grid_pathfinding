use criterion::{black_box, criterion_group, criterion_main, Criterion};
use grid_pathfinding::PathingGrid;
use grid_util::grid::Grid;
use grid_util::point::Point;
use rand::prelude::*;

fn random_grid<const D: bool>(n: usize, rng: &mut StdRng) -> PathingGrid<D> {
    let mut pathing_grid: PathingGrid<D>  = PathingGrid::<D>::new(n, n, false);
    for x in 0..pathing_grid.width() {
        for y in 0..pathing_grid.height() {
            pathing_grid.set(x, y, rng.gen_bool(0.4))
        }
    }
    pathing_grid.generate_components();
    pathing_grid
}
fn random_grid_point<const D: bool>(grid: &PathingGrid<D>, rng: &mut StdRng) -> Point{
    Point::new(rng.gen_range(0..grid.width()) as i32,rng.gen_range(0..grid.height()) as i32)
}

fn test<const D: bool>(pathing_grid: &PathingGrid<D>, start: Point, end: Point) -> Option<Vec<Point>> {
    pathing_grid.get_path_single_goal(start, end, false)
}

fn criterion_benchmark<const D: bool>(c: &mut Criterion) {
    const N: usize = 64;
    const N_GRIDS: usize = 1000;
    const N_PAIRS: usize = 1000;
    let mut rng = StdRng::seed_from_u64(0);
    let mut random_grids: Vec<PathingGrid<D>> = Vec::new();
    for _ in 0..N_GRIDS{
        random_grids.push(random_grid(N, &mut rng))
    }
    
    let start = Point::new(0, 0);
    let end = Point::new(N as i32 - 1, N as i32 - 1);
    c.bench_function("1000 random 64x64 grids", |b| {
        b.iter(|| 
            for grid in &random_grids{
             test(grid, start, end);
            }
        )
    }); 
    let grid = &random_grids[0];
    let mut random_pairs: Vec<(Point,Point)> = Vec::new();
    for _ in 0..N_PAIRS{
        random_pairs.push((random_grid_point(&grid, &mut rng),random_grid_point(&grid, &mut rng)))
    }
    c.bench_function("1000 random start goal pairs on a 64x64 grid", |b| {
        b.iter(|| 
            for (start, end) in &random_pairs{
             test(&grid, start.clone(), end.clone());
            }
        )
    }); 
}

criterion_group!(benches, criterion_benchmark<true>);
criterion_main!(benches);
