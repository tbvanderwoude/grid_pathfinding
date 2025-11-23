use std::i32;

use crate::solver::dijkstra::DijkstraSolver;
use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};
use grid_util::{Grid, Point, SimpleGrid, SimpleValueGrid, ValueGrid};
use smallvec::SmallVec;
use std::iter::zip;

#[derive(Clone, Debug)]
pub struct ALTSolver {
    landmarks: Vec<Point>,
    landmark_distances: SimpleGrid<SmallVec<[i32; 32]>>,
}
impl ALTSolver {
    pub fn new<const ALLOW_DIAGONAL: bool>(
        landmarks: Vec<Point>,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> ALTSolver {
        let d = DijkstraSolver;
        let mut landmark_distances_flipped = Vec::new();
        for landmark in &landmarks {
            let mut dists = SimpleValueGrid::new(grid.width(), grid.height(), i32::MAX);
            let mut ct = grid.context.lock().unwrap();
            ct.astar_jps(
                &landmark,
                |parent, node| d.successors(grid, *parent, node, &|_| false),
                |_| 0,
                |_| false,
            )
            .map(|(v, _c)| v);
            for (p, (_, c)) in &ct.parents {
                dists.set_point(*p, *c);
            }
            landmark_distances_flipped.push(dists);
        }
        let mut landmark_distances = SimpleGrid::new(grid.width(), grid.height(), SmallVec::new());
        for grid in landmark_distances_flipped {
            for x in 0..grid.width() as i32 {
                for y in 0..grid.height() as i32 {
                    landmark_distances
                        .get_mut(x, y)
                        .unwrap()
                        .push(grid.get(x, y));
                }
            }
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
        _: &PathingGrid<ALLOW_DIAGONAL>,
        p1: &Point,
        p2: &Point,
    ) -> i32 {
        let g1 = self.landmark_distances.get_point(*p1).unwrap();
        let g2 = self.landmark_distances.get_point(*p2).unwrap();
        zip(g1, g2).map(|(x, y)| (x - y).abs()).max().unwrap()
    }
}
