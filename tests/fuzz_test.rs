/// Fuzzes pathfinding system by checking for many random grids that a path is always found if the goal is reachable
/// by being part of the same connected component. All system settings (diagonals, improved pruning) are tested.
use grid_pathfinding::{
    pathing_grid::PathingGrid,
    solver::{alt::ALTSolver, astar::AstarSolver, jps::JPSSolver, GridSolver},
    ALLOW_CORNER_CUTTING,
};
use grid_util::*;
use rand::prelude::*;
use smallvec::{smallvec, SmallVec};

fn random_grid<const ALLOW_DIAGONAL: bool>(
    w: usize,
    h: usize,
    rng: &mut StdRng,
) -> PathingGrid<ALLOW_DIAGONAL> {
    let mut pathing_grid: PathingGrid<ALLOW_DIAGONAL> = PathingGrid::new(w, h, false);
    for x in 0..pathing_grid.width() as i32 {
        for y in 0..pathing_grid.height() as i32 {
            pathing_grid.set(x, y, rng.random_bool(0.4))
        }
    }
    pathing_grid.generate_components();
    pathing_grid
}

fn visualize_grid<const ALLOW_DIAGONAL: bool>(
    grid: &PathingGrid<ALLOW_DIAGONAL>,
    start: &Point,
    end: &Point,
) {
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

fn reachable_fuzzer<const ALLOW_DIAGONAL: bool, S, FS>(create_solver: FS)
where
    S: GridSolver,
    FS: Fn(&mut PathingGrid<ALLOW_DIAGONAL>) -> S,
{
    const N: usize = 10;
    const N_GRIDS: usize = 10000;
    let mut rng = StdRng::seed_from_u64(0);
    let mut random_grids: Vec<PathingGrid<ALLOW_DIAGONAL>> = Vec::new();
    for _ in 0..N_GRIDS {
        random_grids.push(random_grid(N, N, &mut rng))
    }

    let start = Point::new(0, 0);
    let end = Point::new(N as i32 - 1, N as i32 - 1);
    for mut random_grid in random_grids {
        random_grid.set_point(start, false);
        random_grid.set_point(end, false);
        let solver = create_solver(&mut random_grid);
        let reachable = random_grid.reachable(&start, &end);
        let path = solver.get_path_single_goal(&mut random_grid, start, end);
        // Show the grid if a path is not found
        if path.is_some() != reachable {
            visualize_grid(&random_grid, &start, &end);
        }
        assert!(path.is_some() == reachable);
    }
}

fn distance_fuzzer<const ALLOW_DIAGONAL: bool>() {
    const N: usize = 10;
    const N_GRIDS: usize = 10000;
    let tolerance = 0.001;

    let mut rng = StdRng::seed_from_u64(0);
    let astar_solver = AstarSolver::new();
    let arr: SmallVec<[bool; 2]> = if ALLOW_DIAGONAL {
        smallvec![false, true]
    } else {
        smallvec![false]
    };
    for improved_pruning in arr {
        let mut random_grids: Vec<PathingGrid<ALLOW_DIAGONAL>> = Vec::new();
        for _ in 0..N_GRIDS {
            random_grids.push(random_grid(N, N, &mut rng))
        }

        let start = Point::new(0, 0);
        let end = Point::new(N as i32 - 1, N as i32 - 1);
        for mut random_grid in random_grids {
            let mut jps_solver = JPSSolver::new(&random_grid, improved_pruning);
            random_grid.set_point(start, false);
            random_grid.set_point(end, false);
            jps_solver.initialize(&random_grid);
            let reachable = random_grid.reachable(&start, &end);
            if reachable {
                let jps_path = jps_solver
                    .get_path_single_goal(&mut random_grid, start, end)
                    .unwrap();
                let astar_path = astar_solver
                    .get_path_single_goal(&mut random_grid, start, end)
                    .unwrap();

                let astar_cost = astar_solver.get_path_cost_float::<ALLOW_DIAGONAL>(&astar_path);
                let jps_cost = jps_solver.get_path_cost_float::<ALLOW_DIAGONAL>(&jps_path);
                if astar_cost >= tolerance {
                    let delta_dist = (jps_cost - astar_cost).abs() / astar_cost;
                    if delta_dist >= tolerance {
                        println!("Astar distance: {astar_cost:4}; JPS distance: {jps_cost:4}");
                        println!("diagonal: {ALLOW_DIAGONAL}; improved_pruning: {improved_pruning}; corner_cutting: {ALLOW_CORNER_CUTTING}");

                        let mut problem_start: Point = start;
                        for (idx, &p) in jps_path.iter().enumerate().rev() {
                            let jps_suffix = &jps_path[idx..].to_vec();
                            let jps_suffix_cost =
                                jps_solver.get_path_cost_float::<ALLOW_DIAGONAL>(jps_suffix);
                            let astar_suffix_path = astar_solver
                                .get_path_single_goal(&mut random_grid, p, end)
                                .expect("A* should find a path from intermediate JPS node");
                            let astar_suffix_cost = astar_solver
                                .get_path_cost_float::<ALLOW_DIAGONAL>(&astar_suffix_path);

                            let rel_diff = (jps_suffix_cost - astar_suffix_cost).abs()
                                / astar_suffix_cost.max(1e-6);

                            // First point where JPS suffix is no longer optimal
                            if rel_diff >= tolerance {
                                println!(
                                    "=> First suboptimal JPS step at index {idx}, point {p:?}"
                                );
                                println!("- JPS suffix from here: {jps_suffix:?}");
                                println!("- A* optimal suffix:    {astar_suffix_path:?}");
                                problem_start = p;
                                break;
                            }
                        }

                        visualize_grid(&random_grid, &problem_start, &end);
                    }

                    assert!(delta_dist < tolerance);
                }
            }
        }
    }
}

fn fuzz_reachable_jps_variants<const ALLOW_DIAGONAL: bool>() {
    let arr: SmallVec<[bool; 2]> = if ALLOW_DIAGONAL {
        smallvec![false, true]
    } else {
        smallvec![false]
    };
    for improved_pruning in arr {
        reachable_fuzzer::<ALLOW_DIAGONAL, _, _>(|pathing_grid| {
            let mut solver = JPSSolver::new(&pathing_grid, improved_pruning);
            solver.initialize(&pathing_grid);
            solver
        })
    }
}

#[test]
fn fuzz_reachable_jps() {
    fuzz_reachable_jps_variants::<false>()
}

#[test]
fn fuzz_reachable_jps_diagonal() {
    fuzz_reachable_jps_variants::<true>()
}

#[test]
fn fuzz_reachable_alt() {
    reachable_fuzzer::<false, _, _>(|pathing_grid| {
        ALTSolver::new_greedy(Point::new(0, 0), 2, pathing_grid)
    });
    reachable_fuzzer::<true, _, _>(|pathing_grid| {
        ALTSolver::new_greedy(Point::new(0, 0), 2, pathing_grid)
    })
}

#[test]
fn fuzz_distance() {
    distance_fuzzer::<false>()
}

#[test]
fn fuzz_distance_diagonal() {
    distance_fuzzer::<true>()
}
