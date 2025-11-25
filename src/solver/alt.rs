use std::i32;

use crate::solver::dijkstra::DijkstraSolver;
use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};
use grid_util::{Grid, Point, SimpleGrid, ValueGrid};
use smallvec::SmallVec;
use std::iter::zip;

#[derive(Clone, Debug)]
pub struct ALTSolver {
    landmarks: Vec<Point>,
    landmark_distances: SimpleGrid<SmallVec<[i32; 32]>>,
}
impl ALTSolver {
    pub fn new_from_landmarks<const ALLOW_DIAGONAL: bool>(
        landmarks: Vec<Point>,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> ALTSolver {
        let d = DijkstraSolver;
        let mut landmark_distances = SimpleGrid::new(grid.width(), grid.height(), SmallVec::new());
        for landmark in &landmarks {
            let mut ct = grid.context.lock().unwrap();
            ct.astar_jps(
                &landmark,
                |parent, node| d.successors(grid, *parent, node, &|_| false),
                |_| 0,
                |_| false,
            );
            for (p, (_, c)) in &ct.parents {
                landmark_distances.get_point_mut(*p).unwrap().push(*c);
            }
        }
        ALTSolver {
            landmarks,
            landmark_distances,
        }
    }
    pub fn new_greedy<const ALLOW_DIAGONAL: bool>(
        initial_landmark: Point,
        landmark_count: usize,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> ALTSolver {
        let d = DijkstraSolver;
        let mut landmarks = vec![initial_landmark];
        let mut landmark_distances = SimpleGrid::new(grid.width(), grid.height(), SmallVec::new());
        while landmarks.len() < landmark_count {
            let landmark = landmarks.last().unwrap();
            let mut ct = grid.context.lock().unwrap();
            ct.astar_jps(
                &landmark,
                |parent, node| d.successors(grid, *parent, node, &|_| false),
                |_| 0,
                |_| false,
            );
            for (p, (_, c)) in &ct.parents {
                landmark_distances.get_point_mut(*p).unwrap().push(*c);
            }
            let mut max_point = *landmark;
            let mut max_cost = 0;
            for x in 0..grid.width() as i32 {
                for y in 0..grid.height() as i32 {
                    // Look to maximize the minimal distance to any existing landmark for a greedy choice
                    let min_c = *landmark_distances
                        .get(x, y)
                        .unwrap()
                        .iter()
                        .min()
                        .unwrap_or(&0);
                    if min_c > max_cost {
                        max_point = Point::new(x, y);
                        max_cost = min_c;
                    }
                }
            }
            // println!("Adding new point greedily: {max_point:?}, cost: {max_cost}");
            landmarks.push(max_point);
        }
        ALTSolver {
            landmarks,
            landmark_distances,
        }
    }
}

impl GridSolver for ALTSolver {
    type Successors = SmallVec<[(Point, i32); N_SMALLVEC_SIZE]>;

    fn successors<const ALLOW_DIAGONAL: bool, F>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        _parent: Option<&Point>,
        node: &Point,
        _goal: &F,
    ) -> Self::Successors
    where
        F: Fn(&Point) -> bool,
    {
        grid.neighborhood_points_and_cost(node)
    }

    fn heuristic<const ALLOW_DIAGONAL: bool>(
        &self,
        grid: &PathingGrid<ALLOW_DIAGONAL>,
        p1: &Point,
        p2: &Point,
    ) -> i32 {
        let g1 = self.landmark_distances.get_point(*p1).unwrap();
        let g2 = self.landmark_distances.get_point(*p2).unwrap();
        zip(g1, g2)
            .map(|(x, y)| (x - y).abs())
            .max()
            .unwrap()
            .max(self.cost(grid, p1, p2))
    }
}
