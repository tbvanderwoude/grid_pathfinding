/// Fuzzes pathfinding system by checking for many random grids that a path is always found if the goal is reachable
/// by being part of the same connected component. All system settings (diagonals, improved pruning) are tested.
use grid_pathfinding::{
    pathing_grid::PathingGrid,
    solver::{astar::AstarSolver, jps::JPSSolver, GridSolver},
};
use grid_util::*;
use rand::prelude::*;

fn random_grid(w: usize, h: usize, rng: &mut StdRng, diagonal: bool) -> PathingGrid {
    let mut pathing_grid: PathingGrid = PathingGrid::new(w, h, false);
    pathing_grid.allow_diagonal_move = diagonal;
    for x in 0..pathing_grid.width() as i32 {
        for y in 0..pathing_grid.height() as i32 {
            pathing_grid.set(x, y, rng.gen_bool(0.4))
        }
    }
    pathing_grid.generate_components();
    pathing_grid
}

fn visualize_grid(grid: &PathingGrid, start: &Point, end: &Point) {
    let grid = &grid.grid;
    for y in (0..grid.height as i32).rev() {
        for x in 0..grid.width as i32 {
            let p = Point::new(x, y);
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
            random_grids.push(random_grid(N, N, &mut rng, diagonal))
        }

        let start = Point::new(0, 0);
        let end = Point::new(N as i32 - 1, N as i32 - 1);
        for mut random_grid in random_grids {
            let mut solver = JPSSolver::new(&random_grid, improved_pruning);
            random_grid.set_point(start, false);
            random_grid.set_point(end, false);
            solver.set_all_jumppoints(&random_grid);
            let reachable = random_grid.reachable(&start, &end);
            let path = solver.get_path_single_goal(&mut random_grid, start, end, false);
            // Show the grid if a path is not found
            if path.is_some() != reachable {
                visualize_grid(&random_grid, &start, &end);
            }
            assert!(path.is_some() == reachable);
        }
    }
}

#[test]
fn fuzz_distance() {
    const N: usize = 5;
    const N_GRIDS: usize = 10000;
    let mut rng = StdRng::seed_from_u64(0);
    let astar_solver = AstarSolver::new();

    for (diagonal, improved_pruning) in [(false, false), (true, false)] {
        let mut random_grids: Vec<PathingGrid> = Vec::new();
        for _ in 0..N_GRIDS {
            random_grids.push(random_grid(N, N, &mut rng, diagonal))
        }

        let start = Point::new(0, 0);
        let end = Point::new(N as i32 - 1, N as i32 - 1);
        for mut random_grid in random_grids {
            let mut jps_solver = JPSSolver::new(&random_grid, improved_pruning);
            random_grid.set_point(start, false);
            random_grid.set_point(end, false);
            jps_solver.set_all_jumppoints(&random_grid);
            let reachable = random_grid.reachable(&start, &end);
            if reachable {
                let jps_path = jps_solver
                    .get_path_single_goal(&mut random_grid, start, end, false)
                    .unwrap();
                let astar_path = astar_solver
                    .get_path_single_goal(&mut random_grid, start, end, false)
                    .unwrap();

                let astar_cost = astar_solver.get_path_cost_float(&astar_path, &random_grid);
                let jps_cost = jps_solver.get_path_cost_float(&jps_path, &random_grid);
                if astar_cost >= 0.01 {
                    let delta_dist = (jps_cost - astar_cost).abs() / astar_cost;
                    if delta_dist >= 0.01 {
                        println!("Astar distance: {astar_cost}; JPS distance: {jps_cost}");
                        println!("Astar path: {astar_path:?}\n JPS path: {jps_path:?}\n");
                        let grid_diag = random_grid.allow_diagonal_move;
                        println!("diagonal: {diagonal}; grid_diag: {grid_diag}; improved_pruning: {improved_pruning}");

                        visualize_grid(&random_grid, &start, &end);
                    }
                    assert!(delta_dist < 0.01);
                }
            }
        }
    }
}
