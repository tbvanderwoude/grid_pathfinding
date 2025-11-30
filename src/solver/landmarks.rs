use core::panic;

use crate::solver::dijkstra::DijkstraSolver;
use crate::{pathing_grid::PathingGrid, solver::GridSolver};
use grid_util::{Grid, Point, SimpleGrid, ValueGrid};
use smallvec::SmallVec;
use std::iter::zip;

#[derive(Clone, Debug)]
pub struct LandmarkMap {
    landmarks: Vec<Point>,
    landmark_distances: SimpleGrid<SmallVec<[i32; 16]>>,
}
impl LandmarkMap {
    pub fn len(&self) -> usize {
        self.landmarks.len()
    }
    #[inline(always)]
    pub fn triangle_heuristic(&self, p1: &Point, p2: &Point) -> i32 {
        let g1 = self.landmark_distances.get_point(*p1).unwrap();
        let g2 = self.landmark_distances.get_point(*p2).unwrap();
        zip(g1, g2).map(|(x, y)| (x - y).abs()).max().unwrap()
    }

    pub fn new_from_landmarks<const ALLOW_DIAGONAL: bool>(
        landmarks: Vec<Point>,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> LandmarkMap {
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
        LandmarkMap {
            landmarks,
            landmark_distances,
        }
    }
    pub fn new_minmax_greedy<const ALLOW_DIAGONAL: bool>(
        initial_landmark: Point,
        landmark_count: usize,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> LandmarkMap {
        let d = DijkstraSolver;
        let mut landmarks = vec![];
        let mut landmark_distances = SimpleGrid::new(grid.width(), grid.height(), SmallVec::new());

        // Look to maximize the minimal distance to any existing landmark for a greedy choice
        while landmarks.len() < landmark_count {
            if landmarks.len() > 0 {
                let mut max_point = None;
                let mut max_cost = 0;
                for x in 0..grid.width() as i32 {
                    for y in 0..grid.height() as i32 {
                        let min_c = *landmark_distances
                            .get(x, y)
                            .unwrap()
                            .iter()
                            .min()
                            .unwrap_or(&0);
                        if min_c > max_cost {
                            max_point = Some(Point::new(x, y));
                            max_cost = min_c;
                        }
                    }
                }
                match max_point {
                    Some(p) => landmarks.push(p),
                    None => {
                        panic!("No point that is maximally far away from existing ones was found")
                    }
                }
            } else {
                landmarks.push(initial_landmark);
            }
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
        }
        LandmarkMap {
            landmarks,
            landmark_distances,
        }
    }
}
