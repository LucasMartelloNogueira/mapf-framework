* Created at: 2026-08-30 10:18:43 -03
* Author: lucas
* Last updated at: 2026-08-30 10:25:46 -03
* AI model used: GPT-5 (Codex)

# Plan: Solution Collision Reporting

## Prompt

```text
leia o arquivo .github/ai-instructions.md e crie um novo plano para implementar a seguinte funcionalidade:

em src/mapf/utils.cpp, a função validateSolution é usada para validar uma solução do MAPF. Entretanto, desejo transformar ela em duas:
1) A função validateSolution atual deve mudar de nome para getCollision. Ela deve retornar todos os conflitos de vértice ou de arestas em um timestep. Mude o nome das variáveis atuais para elas serem somente variáveis de consulta (lookup), crie outras variáveis para armazenar os conflitos. a lista de conflitos de vértices deve retornar um std::vector<mapf::CellConflicts> e a lista de conflitos de arestas devem retornar um std::vector<mapf::EdgeConflict>

2) a função validateSolution deve chamar a função getCollision. Se ambos os vetores de conflitos estiverem vazios, retornar true, caso contrario, retorna false
```

## Context

The repository is a C++20 MAPF framework built with CMake. `validateSolution` is currently declared in `include/mapf/utils.hpp`, implemented in `src/mapf/utils.cpp`, and used by `src/main.cpp` and `src/mapf/solvers/priority_planning_solver.cpp`.

The MAPF context in `.github/mapf.md` defines discrete-time simultaneous execution, vertex conflicts, edge conflicts, swapping conflicts, and stay-at-target behavior. The current validation code detects:

- vertex conflicts by checking whether two agents occupy the same cell at the same timestep;
- edge swap conflicts by checking whether a later path traverses an edge already traversed in the opposite direction at the same timestep;
- stay-at-target conflicts through a goal-cell lookup added to `src/mapf/utils.cpp`.

The requested feature should separate collision discovery from boolean validation:

- `getCollision` gathers and returns collision details;
- `validateSolution` remains the simple validity predicate and delegates to `getCollision`.

## Existing Codebase Analysis

### Current validation flow

`validateSolution` currently uses local lookup containers and returns immediately on the first detected conflict. This is not compatible with the new requirement because `getCollision` must return all detected conflicts.

Current lookup-style data structures:

- `verticeColisions`: stores `cell-position + time` keys for vertex occupancy.
- `edgeColisions`: stores directed edge traversal keys for edge conflict checks.
- `goalVerticeColisions`: stores final vertices occupied under stay-at-target semantics.

These should be renamed to make their lookup role explicit and avoid confusing them with actual returned collision lists.

Suggested names:

```cpp
std::unordered_map<std::string, VertexOccupancy> vertexLookup;
std::unordered_map<std::string, EdgeTraversal> edgeLookup;
std::unordered_map<std::string, GoalOccupancy> goalVertexLookup;
```

The exact helper structs can stay private to `src/mapf/utils.cpp`.

### Existing conflict structs

The repository currently has:

- `include/mapf/core/cell_conflict.hpp`
- `include/mapf/core/edge_conflict.hpp`

`mapf::EdgeConflict` already matches the requested edge conflict type name:

```cpp
namespace mapf {
    struct EdgeConflict {
        const Cell& cell_1;
        const Cell& cell_2;
        const int time;
    };
}
```

The vertex conflict type is `mapf::CellConflict`:

```cpp
namespace mapf {
    struct CellConflict {
        const Cell& cell;
        const int time;
    };
}
```

The implementation must use only `CellConflict`. The plural spelling from the original prompt was a typo and must not be introduced as a type or alias.

### Public return shape

Returning two vectors directly requires a grouped return type. The cleanest public interface is a small result struct:

```cpp
namespace mapf {
    struct SolutionConflicts {
        std::vector<CellConflict> cellConflicts;
        std::vector<EdgeConflict> edgeConflicts;
    };
}
```

`getCollision` can then return `mapf::SolutionConflicts`.

## Documentation And Libraries Researched

No third-party library is required.

Use only existing C++20 standard-library facilities:

- `<vector>` for returned conflict lists.
- `<unordered_map>` for lookup tables that also retain the previous occupant/traversal needed to report a conflict.
- `<string>` or private key structs for lookup keys.
- `<sstream>` or `std::format` for generating readable keys, with C++20 placeholder syntax if `std::format` is kept.

Important implementation note: current code uses `std::format("%d-%d", ...)`, but `std::format` requires `{}` placeholders. The implementation should either use `std::format("{}-{}", x, y)` or reuse the existing stream-based key helpers to avoid incorrect keys.

## Files To Change Or Create

### `include/mapf/utils.hpp`

Update the public interface:

- include `mapf/core/cell_conflict.hpp`;
- include `mapf/core/edge_conflict.hpp`;
- declare the grouped return type, either in this header or in a new core header;
- declare `getCollision`;
- keep `validateSolution`.

Recommended declarations:

```cpp
#include "mapf/core/cell_conflict.hpp"
#include "mapf/core/edge_conflict.hpp"

namespace mapf {
    struct SolutionConflicts {
        std::vector<CellConflict> cellConflicts;
        std::vector<EdgeConflict> edgeConflicts;
    };
}

mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths);

bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths);
```

### `src/mapf/utils.cpp`

Refactor the current `validateSolution` implementation into `getCollision`:

- move the current validation traversal into `getCollision`;
- rename lookup variables so they no longer sound like returned conflicts;
- create separate returned vectors for actual conflicts;
- collect every conflict instead of returning on the first conflict;
- have `validateSolution` call `getCollision`.

Recommended structure:

```cpp
mapf::SolutionConflicts getCollision(const std::vector<std::list<mapf::Cell*>>& paths) {
    mapf::SolutionConflicts conflicts;

    // lookup containers used only to detect repeated occupation/traversal
    std::unordered_map<std::string, mapf::Cell*> vertexLookup;
    std::unordered_map<std::string, EdgeTraversal> edgeLookup;
    std::unordered_map<std::string, GoalOccupancy> goalVertexLookup;

    // populate conflicts.cellConflicts and conflicts.edgeConflicts

    return conflicts;
}

bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths) {
    mapf::SolutionConflicts conflicts = getCollision(paths);
    return conflicts.cellConflicts.empty() && conflicts.edgeConflicts.empty();
}
```

Implementation details:

- Store enough data in lookups to report a conflict after detection.
- Use `path.size() - 1` as the arrival time at a goal, not the post-loop value after `t++`.
- For stay-at-target conflicts, compare another agent's `(cell, time)` against previously completed goal cells. If the other agent reaches that goal at time `t` and the first agent's goal arrival time is `<= t`, append a vertex conflict for that cell and `t`.
- For normal vertex conflicts, append a vertex conflict when two agents occupy the same cell at the same timestep.
- For edge conflicts, preserve existing swap-conflict behavior by detecting opposite directed traversals at the same timestep.
- If same-direction edge conflicts should also be reported, this should be explicitly added to the spec because `.github/mapf.md` describes same-direction edge conflicts, while the current code primarily detects swaps.

### `include/mapf/core/cell_conflict.hpp`

No plural `CellConflicts` type or alias should be added. Keep `CellConflict` as the only public vertex conflict type.

### `include/mapf/core/solution_conflicts.hpp`

Optional new header if `SolutionConflicts` should not live in `utils.hpp`.

Recommended if conflict reporting will be reused outside utility validation:

```cpp
#pragma once

#include <vector>

#include "cell_conflict.hpp"
#include "edge_conflict.hpp"

namespace mapf {
    struct SolutionConflicts {
        std::vector<CellConflict> cellConflicts;
        std::vector<EdgeConflict> edgeConflicts;
    };
}
```

### `src/main.cpp`

No production change is required unless the manual test should print collision details. Existing calls to `validateSolution` should continue to compile.

### `src/mapf/solvers/priority_planning_solver.cpp`

No behavioral change is required. Existing calls to `validateSolution` should continue to compile and return the same boolean result.

## Implementation Steps

1. Add or expose the conflict return type.
2. Use the existing `CellConflict` type as the only vertex conflict type.
3. Update `include/mapf/utils.hpp` with `getCollision` and the conflict result type.
4. Refactor `src/mapf/utils.cpp` so the old `validateSolution` traversal becomes `getCollision`.
5. Rename lookup containers and variables to clarify their lookup-only role.
6. Replace early `return false` conflict exits with pushes into the appropriate conflict vectors.
7. Correct key generation so vertex lookup keys use actual coordinates.
8. Correct goal arrival time to use the last path timestep.
9. Reimplement `validateSolution` as a thin wrapper over `getCollision`.
10. Build the project with CMake and run the current executable or a focused manual scenario.

## Collision Semantics To Preserve

- Time starts at `0` for the first cell in each path.
- A path with cells `[A, B, C]` reaches `C` at time `2`.
- Under stay-at-target semantics, after an agent reaches its final cell at time `t_goal`, that cell remains occupied for all `t >= t_goal`.
- `validateSolution` returns `true` only when both returned conflict vectors are empty.
- `validateSolution` returns `false` when at least one vertex or edge conflict is returned.

## Acceptance Criteria

- `getCollision` is publicly declared and implemented.
- `validateSolution` calls `getCollision` instead of duplicating collision logic.
- `getCollision` returns all detected vertex conflicts in `std::vector<mapf::CellConflict>`.
- `getCollision` returns all detected edge conflicts in `std::vector<mapf::EdgeConflict>`.
- Lookup variables are named as lookups, not as collision result containers.
- Actual conflict result vectors are distinct from lookup containers.
- Existing callers of `validateSolution` continue to compile.
- A solution with no conflicts returns two empty conflict vectors and validates as `true`.
- A vertex conflict at a simultaneous timestep appears in the vertex conflict vector.
- An edge swap conflict appears in the edge conflict vector.
- A stay-at-target conflict appears in the vertex conflict vector at the timestep when the later agent enters the occupied goal cell.
- The manual scenario where one agent reaches `(4, 2)` at time `2` and another enters `(4, 2)` at time `4` reports the conflict at time `4`.
- `cmake --build build` succeeds.

## TODO

TODO: Decide in the spec whether same-direction edge conflicts from `.github/mapf.md` should be collected in addition to the current swap-conflict validation behavior.

## Adjustments

### adjustment 1

* Datetime: 2026-08-30 10:25:46 -03

#### Prompt

```text
add the following adjusment to the Adjusment sectin in specs/003-solution-collision-reporting/plan.md and also edit the plan. CellConflicts was a typo, use only CellConflict, do not use CellConflicts
```

#### Changes

- Before: The plan treated `CellConflicts` as a possible compatibility alias for `CellConflict`.
- After: The plan states that `CellConflicts` was a typo and must not be added as a type or alias.
- Before: Acceptance criteria allowed a generic vector of the project's cell conflict type.
- After: Acceptance criteria explicitly require `std::vector<mapf::CellConflict>`.
- Before: Implementation step 2 asked the spec to decide between plural alias support and the existing singular type.
- After: Implementation step 2 requires using only the existing `CellConflict` type.
