//! # grid_pathfinding
//!
//! A complete grid-based pathfinding system. Implements
//! [Jump Point Search](https://en.wikipedia.org/wiki/Jump_point_search) for speedy optimal
//! pathfinding. Pre-computes
//! [connected components](https://en.wikipedia.org/wiki/Component_(graph_theory))
//! to avoid flood-filling behaviour if no path exists.
pub mod path_graph;
pub mod astar_jps;
