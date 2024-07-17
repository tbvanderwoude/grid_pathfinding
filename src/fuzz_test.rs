/// Fuzzes pathfinding system by checking for many random grids that a path is always found if the goal is reachable
/// by being part of the same connected component. All system settings (diagonals, improved pruning) are tested.

use super::*;
use rand::prelude::*;

fn random_grid(n: usize, rng: &mut StdRng, diagonal: bool, improved_pruning: bool) -> PathingGrid {
    let mut pathing_grid: PathingGrid = PathingGrid::new(n, n, false);
    pathing_grid.allow_diagonal_move = diagonal;
    pathing_grid.improved_pruning = improved_pruning;
    for x in 0..pathing_grid.width() {
        for y in 0..pathing_grid.height() {
            pathing_grid.set(x, y, rng.gen_bool(0.4))
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

fn visualize_grid(grid: &PathingGrid, start: &Point, end: &Point) {
    let grid = &grid.grid;
    for y in (0..grid.height).rev() {
        for x in 0..grid.width {
            let p = Point::new(x as i32, y as i32);
            if *start == p {
                print!("S");
            } else if *end == p {
                print!("G");
            } else if grid.get(x, y) {
                print!("#");
            } else {
                print!(".");
            }
        }
        println!();
    }
}

#[test]
fn fuzz() {
    const N: usize = 10;
    const N_GRIDS: usize = 10000;
    let mut rng = StdRng::seed_from_u64(0);
    for (diagonal, improved_pruning) in [(false, false), (true, false), (true, true)] {
        let mut random_grids: Vec<PathingGrid> = Vec::new();
        for _ in 0..N_GRIDS {
            random_grids.push(random_grid(N, &mut rng, diagonal, improved_pruning))
        }

        let start = Point::new(0, 0);
        let end = Point::new(N as i32 - 1, N as i32 - 1);
        for mut random_grid in random_grids {
            random_grid.set_point(start, false);
            random_grid.set_point(end, false);
            let reachable = random_grid.reachable(&start, &end);
            let path = random_grid.get_path_single_goal(start, end, false);
            // Show the grid if a path is not found
            if path.is_some() != reachable {
                visualize_grid(&random_grid, &start, &end);
            }
            assert!(path.is_some() == reachable);
        }
    }
}
