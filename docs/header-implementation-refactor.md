# Header and Implementation Organization

The framework now keeps public interfaces in `include/` and method implementations in `src/`.

Core grid behavior remains in `mapf::Grid`, while pathfinding behavior is exposed through solver classes:

- `mapf::AStarSolver` in `include/mapf/pathfinding/a_star.hpp`
- `mapf::AStarSippSolver` in `include/mapf/pathfinding/a_star_sipp.hpp`

SIPP support structs live in `include/mapf/pathfinding/sipp/`, with method bodies implemented under the matching `src/mapf/pathfinding/sipp/` path.

The project can be built with CMake:

```bash
cmake -S . -B build
cmake --build build
./build/mapf_app
```

External libraries should be added with target-based CMake, using `find_package(...)` and `target_link_libraries(...)` on the `mapf` or `mapf_app` targets.
