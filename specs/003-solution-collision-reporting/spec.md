* Created at: 2026-08-30 10:29:59 -03
* Author: lucas
* Last updated at: 2026-08-30 10:29:59 -03
* AI model used: GPT-5 (Codex)

# Spec: Solution Collision Reporting

## Objective

Split MAPF solution validation into two responsibilities:

- `getCollision` detects and returns all known solution conflicts.
- `validateSolution` remains a boolean predicate implemented by calling `getCollision`.

The behavior must preserve the current validation semantics while allowing callers to inspect collision details.

## Scope

This feature covers collision reporting for complete MAPF path sets represented as:

```cpp
std::vector<std::list<mapf::Cell*>>
```

The implementation must report:

- vertex conflicts, where two agents occupy the same cell at the same timestep;
- edge swap conflicts, where two agents traverse the same edge in opposite directions at the same timestep;
- stay-at-target conflicts, where a later agent enters a cell after another agent has already reached that cell as its goal.

Same-direction edge conflicts described in `.github/mapf.md` are outside the scope of this implementation because the current validation code does not detect them. They can be added in a later feature if needed.

## Public API Requirements

### Vertex Conflict Type

Use only the existing singular type:

```cpp
mapf::CellConflict
```

Do not introduce `mapf::CellConflicts`, and do not create a type alias for it.

### Edge Conflict Type

Use the existing type:

```cpp
mapf::EdgeConflict
```

### Collision Result Type

Add a grouped result type that contains both conflict vectors:

```cpp
namespace mapf {
    struct SolutionConflicts {
        std::vector<CellConflict> cellConflicts;
        std::vector<EdgeConflict> edgeConflicts;
    };
}
```

The preferred location is a new header:

```text
include/mapf/core/solution_conflicts.hpp
```

This keeps conflict result data in `mapf/core` instead of embedding a reusable data structure directly in `utils.hpp`.

### Utility Functions

Expose this new function:

```cpp
mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths);
```

Keep this existing function:

```cpp
bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths);
```

`validateSolution` must call `getCollision` and return:

```cpp
conflicts.cellConflicts.empty() && conflicts.edgeConflicts.empty()
```

## Files To Change

### `include/mapf/core/solution_conflicts.hpp`

Create this header with:

- `#pragma once`;
- `#include <vector>`;
- `#include "cell_conflict.hpp"`;
- `#include "edge_conflict.hpp"`;
- `namespace mapf`;
- `struct SolutionConflicts`.

### `include/mapf/utils.hpp`

Update the declarations:

- include `mapf/core/solution_conflicts.hpp`;
- declare `getCollision`;
- keep `validateSolution`.

The resulting declarations should include:

```cpp
mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths);

bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths);
```

### `src/mapf/utils.cpp`

Refactor the current `validateSolution` implementation into `getCollision`.

Use lookup containers only for conflict detection, and use separate vectors inside `mapf::SolutionConflicts` to store reported conflicts.

The current lookup variables must be renamed:

- `verticeColisions` becomes a vertex occupancy lookup.
- `edgeColisions` becomes an edge traversal lookup.
- `goalVerticeColisions` becomes a goal vertex lookup.

Recommended naming:

```cpp
vertexLookup
edgeLookup
goalVertexLookup
```

Fix the current coordinate key bug by avoiding `std::format("%d-%d", ...)`. If `std::format` is used, use C++ format placeholders:

```cpp
std::format("{}-{}", cell->position.x, cell->position.y)
```

It is also acceptable to use the existing stream-based key helpers.

Fix the stay-at-target arrival time bug by storing the last path timestep:

```cpp
arrivalTime = static_cast<int>(path.size()) - 1;
```

Do not store the post-loop `t` value as the arrival time.

### `src/main.cpp`

No required production change.

Existing calls to `validateSolution` must keep compiling. Manual test code may be adjusted later, but this spec does not require output changes in `main`.

### `src/mapf/solvers/priority_planning_solver.cpp`

No required behavioral change.

Existing calls to `validateSolution` must keep compiling and should receive the same boolean answer as before, except where the current implementation had the known stay-at-target time/key bugs.

## Implementation Instructions

1. Create `include/mapf/core/solution_conflicts.hpp`.
2. Update `include/mapf/utils.hpp` to include the new header and declare `getCollision`.
3. In `src/mapf/utils.cpp`, introduce private helper structs if needed to store lookup metadata, such as the previously occupied cell or edge endpoints.
4. Move the current validation traversal into `getCollision`.
5. Replace all early returns on detected conflicts with `push_back` into `conflicts.cellConflicts` or `conflicts.edgeConflicts`.
6. Preserve traversal through all paths so all detectable conflicts are returned.
7. Use lookup variables only as lookup variables. Do not name lookup containers as if they were the returned conflict lists.
8. For each `(cell, timestep)`:
   - check whether a previous goal occupancy blocks that cell at this timestep;
   - check whether another agent already occupied this cell at this timestep;
   - append `mapf::CellConflict{*cell, timestep}` for each detected vertex-style conflict.
9. For each move from `previousCell` to `cell` at timestep `t`:
   - check whether the opposite edge traversal was already seen at the same timestep;
   - append `mapf::EdgeConflict{*previousCell, *cell, t}` when a swap is detected.
10. After finishing each non-empty path:
    - store the final cell in `goalVertexLookup`;
    - store the arrival time as `path.size() - 1`.
11. Implement `validateSolution` as a thin wrapper over `getCollision`.
12. Build the project with CMake.

## Empty Path Behavior

If a path is empty, `getCollision` must skip it.

This avoids dereferencing a null or unset `previousCell`. Empty paths do not directly create conflicts, but callers may still treat empty paths as invalid elsewhere if needed.

## Collision Semantics

- The first cell in a path is occupied at time `0`.
- A path with three cells reaches its final cell at time `2`.
- Stay-at-target means that an agent remains at its final cell for every timestep greater than or equal to its goal arrival time.
- A stay-at-target conflict should be reported at the timestep when the later agent enters or occupies the already reserved goal cell.
- `getCollision` returns all conflicts it detects; it must not stop at the first one.
- `validateSolution` returns `true` only when both returned vectors are empty.

## Acceptance Criteria

- `include/mapf/core/solution_conflicts.hpp` exists and defines `mapf::SolutionConflicts`.
- `mapf::SolutionConflicts::cellConflicts` has type `std::vector<mapf::CellConflict>`.
- `mapf::SolutionConflicts::edgeConflicts` has type `std::vector<mapf::EdgeConflict>`.
- No `mapf::CellConflicts` type or alias is introduced.
- `getCollision` is declared in `include/mapf/utils.hpp`.
- `getCollision` is implemented in `src/mapf/utils.cpp`.
- `validateSolution` calls `getCollision`.
- `validateSolution` returns `true` when both conflict vectors are empty.
- `validateSolution` returns `false` when either conflict vector is non-empty.
- Lookup variable names clearly indicate lookup usage.
- Conflict vectors are separate from lookup containers.
- The current manual scenario where one agent reaches `(4, 2)` at time `2` and another enters `(4, 2)` at time `4` reports a vertex conflict at time `4`.
- Existing callers in `src/main.cpp` and `src/mapf/solvers/priority_planning_solver.cpp` compile without API changes.
- `cmake --build build` succeeds.

## Out Of Scope

- Adding tests or a CTest target.
- Changing `PriorityPlanningSolver` behavior.
- Changing the path representation.
- Reporting agent IDs in conflict structs.
- Adding same-direction edge conflict detection.
- Renaming `CellConflict`.
