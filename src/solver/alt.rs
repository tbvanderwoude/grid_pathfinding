use crate::solver::landmarks::LandmarkMap;
use crate::{pathing_grid::PathingGrid, solver::GridSolver, N_SMALLVEC_SIZE};
use grid_util::Point;
use smallvec::SmallVec;

#[derive(Clone, Debug)]
pub struct ALTSolver {
    landmark_map: LandmarkMap,
}
impl ALTSolver {
    pub fn new_from_landmarks<const ALLOW_DIAGONAL: bool>(
        landmarks: Vec<Point>,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> ALTSolver {
        ALTSolver {
            landmark_map: LandmarkMap::new_from_landmarks(landmarks, grid),
        }
    }
    pub fn new_from_landmark_map(landmark_map: LandmarkMap) -> ALTSolver {
        ALTSolver { landmark_map }
    }
    pub fn new_greedy<const ALLOW_DIAGONAL: bool>(
        initial_landmark: Point,
        landmark_count: usize,
        grid: &mut PathingGrid<ALLOW_DIAGONAL>,
    ) -> ALTSolver {
        ALTSolver {
            landmark_map: LandmarkMap::new_minmax_greedy(initial_landmark, landmark_count, grid),
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

    fn heuristic<const ALLOW_DIAGONAL: bool>(&self, p1: &Point, p2: &Point) -> i32 {
        self.landmark_map
            .triangle_heuristic(p1, p2)
            .max(self.cost::<ALLOW_DIAGONAL>(p1, p2))
    }
}
