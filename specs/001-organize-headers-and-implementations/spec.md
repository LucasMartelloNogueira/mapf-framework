Created at: 2026-08-17 18:44:06 -03
Author: lucas
Last updated at: 2026-08-17 18:44:06 -03
AI model used: GPT-5

# Spec: Organize Headers and Implementations

## Objective

Refactor the MAPF framework so public headers under `include/` expose interfaces, struct definitions, aliases, and method signatures, while implementation logic lives under matching `src/` paths. Add solver classes for A* and SIPP pathfinding, create SIPP support structs as separate headers, and introduce a CMake build system that builds the project executable and can be extended with external libraries.

## Requirements

- `Grid` must keep grid construction, cell lookup, neighbor lookup, Manhattan distance, grid printing, and cell storage access.
- `Grid` must no longer expose `getAStarPath` or `getSippPath`.
- `AStarSolver` must be declared in `include/mapf/pathfinding/a_star.hpp`.
- `AStarSolver::solve` must implement the previous `Grid::getAStarPath` behavior and receive `Grid& grid` as an argument.
- `AStarSippSolver` must be declared in `include/mapf/pathfinding/a_star_sipp.hpp`.
- `AStarSippSolver::getSafeIntervalsByCell` must build the safe interval table from other agents' paths.
- `AStarSippSolver::solve` must implement the previous `Grid::getSippPath` behavior and receive `Grid& grid` as an argument.
- SIPP structs previously declared inside `getSippPath` must become standalone headers under `include/mapf/pathfinding/sipp/`.
- `src/main.cpp` must use the new solver classes.
- `readme.md` must document CMake configure, build, and run commands.

## Build System

- Add a root `CMakeLists.txt`.
- Require CMake 3.20 or newer and C++20.
- Create a `mapf` library target containing framework `.cpp` files.
- Create a `mapf_app` executable target from `src/main.cpp`.
- Link `mapf_app` against `mapf`.
- Configure `include/` as a public include directory on `mapf`.
- Leave the build system ready for external libraries through `find_package(...)` and `target_link_libraries(mapf ...)`.

## Acceptance Criteria

- The project configures with `cmake -S . -B build`.
- The project builds with `cmake --build build`.
- The executable runs with `./build/mapf_app`.
- The existing sample prints a SIPP path.
- Public pathfinding calls use `AStarSolver::solve` and `AStarSippSolver::solve`.
- Public headers no longer contain the moved algorithm implementations.

## Adjustments

No adjustments yet.
