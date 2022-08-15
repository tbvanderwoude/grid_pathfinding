# grid_pathfinding

A grid-based pathfinding system. Implements [Jump Point Search](https://en.wikipedia.org/wiki/Jump_point_search) with 
[improved pruning rules](https://www.researchgate.net/publication/287338108_Improving_jump_point_search) for speedy pathfinding. Pre-computes
[connected components](https://en.wikipedia.org/wiki/Component_(graph_theory))
to avoid flood-filling behaviour if no path exists.

### Examples
See [examples](examples/) for some examples on how to initialize the relevant datastructures and find paths.