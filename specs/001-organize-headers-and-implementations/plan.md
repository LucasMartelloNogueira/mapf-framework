Created at: 2026-08-17 18:29:13 -03
Author: lucas
Last updated at: 2026-08-17 18:29:13 -03

# Plan: Organize Headers and Implementations

## Prompt

```text
leia o arquivo .github/ai-instructions.md e faça um plano para a seguinte funcionalidade

* objetivo: organizar o diretório em pastas especificas de headers e implementações

Para isso, faça as seguintes modificações:
* deixe a pasta include e os arquivos hpp só com as interfaces de classe, definições de struct e assinatura de métodos
* crie, usando o mesmo caminho de arquivos da pasta include, os arquivos de implementações dos hpp
* em include/mapf/pathfinding/a_star.hpp, crie uma classe AStarSolver com um método solve que é basicamente o método getAStarPath de grid.hpp. Faça as adaptações necessárias passando o grid como argumento para o método solve. Faça apenas a interface dessa classe nesse arquivo, crie um arquivo de implementação separadamente dentro da pasta source
* include/mapf/pathfinding/a_star_sipp.hpp, crie uma classe AStarSippSolver com dois métodos: getSafeIntervalsByCell e solve. O método solve é o método getSippPath mas adaptado recebendo o Grid como parametro. Além disso, retire todas as definições de struct de dentro do getSippPath e crie um arquivo hpp para cada struct. Junte toda a lógica para obter o safeIntervalsByCell e bote no método getSafeIntervalsByCell, fazendo os ajustes necessários. Além disso, deixe somente as interfaces no arquivo e crie um arquivo com a implementação em src/mapf/pathfinding/a_star_sipp.cpp
```

## Context

The repository contains a small C++20 MAPF framework using headers under `include/mapf` and implementations/examples under `src`. The context file requested by `.github/ai-instructions.md` was not found at the repository root as `mapf.md`; the available context file is `.github/mapf.md`, which defines the classical MAPF problem, discrete time, move/wait actions, and conflict types such as vertex conflicts and edge swaps.

Current implementation state:

- `include/mapf/core/grid.hpp` contains the full inline implementation of `Grid`, `getAStarPath`, and `getSippPath`.
- `include/mapf/pathfinding/a_star.hpp` exists but is empty.
- `include/mapf/pathfinding/a_star_sipp.hpp` exists but is empty.
- `src/mapf/pathfinding/a_star.cpp` exists but is empty.
- `src/mapf/pathfinding/a_star_sipp.cpp` exists but is empty.
- There is no build system file; `readme.md` currently compiles only `src/main.cpp` with `g++ -std=c++20 -Iinclude src/main.cpp -o bin/main`.

The prompt mentions `source` once, but the repository already uses `src`, and the final SIPP requirement explicitly says `src/mapf/pathfinding/a_star_sipp.cpp`. This plan will use `src` consistently for implementation files.

## Existing Codebase Analysis

The framework currently models grid cells through `Cell` and `Position` structs and keeps grid storage inside `Grid` as `std::vector<Cell> cells`, `rows`, and `cols`.

The current `Grid` responsibilities are mixed:

- Grid construction and cell access.
- Neighborhood and heuristic helpers.
- A* pathfinding.
- SIPP pathfinding.
- Debug printing.

The refactor should separate these responsibilities:

- `Grid` should keep grid topology and helper methods.
- `AStarSolver` should own A* path computation.
- `AStarSippSolver` should own SIPP path computation and safe interval generation.
- Structs used by SIPP should become reusable header-level definitions instead of local structs inside a method.

## Libraries

No new third-party libraries are needed.

Use the C++ standard library already used in the repository:

- `<vector>` for cells, paths converted from lists, and interval arrays.
- `<list>` for the current public path representation.
- `<queue>` for A* and SIPP priority queues.
- `<unordered_map>` and `<unordered_set>` for best-cost tables, closed sets, collision intervals, and blocked edge arrivals.
- `<algorithm>` for sorting, merging, reversing, and min/max operations.
- `<limits>` for SIPP infinite time sentinel.
- `<cmath>` or `<cstdlib>` for Manhattan distance absolute values.
- `<iostream>` only in implementation files that print.

Keeping only the standard library avoids adding build complexity while preserving the current behavior.

## Files To Change

### Existing headers

- `include/mapf/core/grid.hpp`
  - Keep only `Grid` data members and method declarations.
  - Remove inline implementations.
  - Remove `getAStarPath` and `getSippPath` from `Grid`.
  - Add a const or non-const accessor needed by SIPP to iterate all cells, for example `std::vector<Cell>& getCells()` or `const std::vector<Cell>& getCells() const`.

- `include/mapf/core/cell.hpp`
  - Keep only the `Cell` struct and a `printCell(Cell* cell)` declaration.
  - Move the `printCell` implementation to `src/mapf/core/cell.cpp`.
  - Add required includes for declarations only, such as `<iosfwd>` only if needed. Prefer no I/O include in the header if `printCell` only needs a forward declaration.

- `include/mapf/core/a-star-heap.hpp`
  - Keep the `CompareAStarNode` struct declaration/definition and the `using AStarHeap` alias.
  - Since comparator bodies in structs are part of the type and this is a small reusable struct, it can stay in the header as a struct definition. If strict method-body removal is required, declare `bool operator()(...) const;` and move the body to `src/mapf/core/a-star-heap.cpp`.

- `include/mapf/pathfinding/a_star.hpp`
  - Define only the `AStarSolver` class interface.

- `include/mapf/pathfinding/a_star_sipp.hpp`
  - Define only the `AStarSippSolver` class interface.
  - Include the SIPP struct headers needed by the public signatures.

### New implementation files

- `src/mapf/core/grid.cpp`
  - Implement `Grid` constructors, `getCellIndex`, `getCellPtr`, `getNeighbors`, `getManhattanDistance`, `printGrid`, and any new cell accessor.

- `src/mapf/core/cell.cpp`
  - Implement `printCell`.

- `src/mapf/core/a-star-heap.cpp`
  - Optional if the comparator body is moved out of the header.

- `src/mapf/pathfinding/a_star.cpp`
  - Implement `AStarSolver::solve` using the logic currently in `Grid::getAStarPath`, adapted to call `grid.getNeighbors(...)` and `grid.getManhattanDistance(...)`.

- `src/mapf/pathfinding/a_star_sipp.cpp`
  - Implement `AStarSippSolver::getSafeIntervalsByCell`.
  - Implement `AStarSippSolver::solve` using the logic currently in `Grid::getSippPath`, adapted to receive `Grid& grid`.

### New SIPP struct headers

Create one header per struct, as requested:

- `include/mapf/pathfinding/sipp/interval.hpp`
- `include/mapf/pathfinding/sipp/edge_key.hpp`
- `include/mapf/pathfinding/sipp/edge_key_hash.hpp`
- `include/mapf/pathfinding/sipp/state_key.hpp`
- `include/mapf/pathfinding/sipp/state_key_hash.hpp`
- `include/mapf/pathfinding/sipp/sipp_node.hpp`
- `include/mapf/pathfinding/sipp/compare_sipp_node.hpp`
- `include/mapf/pathfinding/sipp/parent_info.hpp`

The SIPP solver also needs to expose edge-blocking information together with safe intervals. Prefer adding a small return/result struct:

- `include/mapf/pathfinding/sipp/safe_interval_table.hpp`

This file can define:

```cpp
using SafeIntervalsByCell = std::unordered_map<Cell*, std::vector<Interval>>;
using BlockedEdgeArrivals = std::unordered_map<EdgeKey, std::unordered_set<int>, EdgeKeyHash>;

struct SafeIntervalTable {
    SafeIntervalsByCell safeIntervalsByCell;
    BlockedEdgeArrivals blockedEdgeArrivals;
};
```

This keeps `getSafeIntervalsByCell` useful for `solve`, because SIPP needs both vertex safe intervals and blocked edge arrivals to reject edge swaps.

### Existing source example

- `src/main.cpp`
  - Replace calls to `grid.getAStarPath(...)` with `AStarSolver::solve(grid, ...)`.
  - Replace calls to `grid.getSippPath(...)` with `AStarSippSolver::solve(grid, ...)`.

### Documentation/build command

- `readme.md`
  - Update the compile command to include all `.cpp` files, for example:

```bash
g++ -std=c++20 -Iinclude \
  src/main.cpp \
  src/mapf/core/grid.cpp \
  src/mapf/core/cell.cpp \
  src/mapf/pathfinding/a_star.cpp \
  src/mapf/pathfinding/a_star_sipp.cpp \
  -o bin/main
```

TODO: Add a proper build system such as CMake once the project grows beyond a few source files.

## Proposed Interfaces

### Grid

```cpp
#pragma once

#include <list>
#include <vector>

#include "cell.hpp"

namespace mapf {

class Grid {
private:
    std::vector<Cell> cells;
    int rows;
    int cols;

public:
    Grid(int rows, int cols);
    Grid(std::vector<std::vector<int>>* free, int rows, int cols);

    int getCellIndex(int x, int y);
    Cell* getCellPtr(int x, int y);
    std::list<Cell*> getNeighbors(Cell* cell);
    int getManhattanDistance(Cell* a, Cell* b);
    void printGrid();

    std::vector<Cell>& getCells();
    const std::vector<Cell>& getCells() const;
};

}
```

### AStarSolver

```cpp
#pragma once

#include <list>

#include "mapf/core/cell.hpp"
#include "mapf/core/grid.hpp"

namespace mapf {

class AStarSolver {
public:
    std::list<Cell*> solve(Grid& grid, Cell* start, Cell* goal);
};

}
```

### AStarSippSolver

```cpp
#pragma once

#include <list>
#include <vector>

#include "mapf/core/cell.hpp"
#include "mapf/core/grid.hpp"
#include "mapf/pathfinding/sipp/safe_interval_table.hpp"

namespace mapf {

class AStarSippSolver {
public:
    SafeIntervalTable getSafeIntervalsByCell(
        Grid& grid,
        const std::vector<std::list<Cell*>>& otherAgentPaths
    );

    std::list<Cell*> solve(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const std::vector<std::list<Cell*>>& otherAgentPaths
    );
};

}
```

## Implementation Strategy

1. Move core `Grid` method bodies from `include/mapf/core/grid.hpp` into `src/mapf/core/grid.cpp`.
2. Remove pathfinding methods from `Grid`.
3. Move `printCell` from `include/mapf/core/cell.hpp` into `src/mapf/core/cell.cpp`.
4. Create the SIPP struct headers, preserving the same fields and equality/hash behavior currently declared inside `getSippPath`.
5. Implement `AStarSolver::solve` in `src/mapf/pathfinding/a_star.cpp`.
6. Implement `AStarSippSolver::getSafeIntervalsByCell` by extracting the collision interval creation, blocked edge arrivals creation, collision interval merge, and safe interval generation from the current `Grid::getSippPath`.
7. Implement `AStarSippSolver::solve` by using the returned `SafeIntervalTable`, then running the remaining SIPP search.
8. Update `src/main.cpp` to instantiate solvers.
9. Update the compile command in `readme.md`.
10. Compile and run the sample executable.

## Important Implementation Notes

- Preserve the current public path type: `std::list<Cell*>`.
- Preserve null and blocked-cell validation behavior:

```cpp
if (start == nullptr || goal == nullptr) {
    return {};
}

if (!start->isFree || !goal->isFree) {
    return {};
}
```

- Preserve the current A* lazy deletion strategy using `bestG`.
- Preserve the current SIPP infinite time sentinel:

```cpp
constexpr int INF_TIME = std::numeric_limits<int>::max() / 4;
```

- Keep edge-swap prevention by preserving `blockedEdgeArrivals`.
- Avoid copying `otherAgentPaths` by taking it as `const std::vector<std::list<Cell*>>&`.
- When iterating all grid cells in SIPP, use the new `Grid::getCells()` accessor instead of direct private member access.
- Prefer `nullptr` over `NULL` when moving implementation code.

## Example Implementation Sketch

### A* solve adaptation

```cpp
std::list<Cell*> AStarSolver::solve(Grid& grid, Cell* start, Cell* goal) {
    if (start == nullptr || goal == nullptr) {
        return {};
    }

    if (!start->isFree || !goal->isFree) {
        return {};
    }

    // Existing Grid::getAStarPath logic goes here.
    // Replace getNeighbors(current.cell) with grid.getNeighbors(current.cell).
    // Replace getManhattanDistance(a, b) with grid.getManhattanDistance(a, b).
}
```

### SIPP safe interval extraction

```cpp
SafeIntervalTable AStarSippSolver::getSafeIntervalsByCell(
    Grid& grid,
    const std::vector<std::list<Cell*>>& otherAgentPaths
) {
    SafeIntervalTable table;

    // Build collision intervals by cell.
    // Build blocked edge arrivals.
    // Merge collision intervals.
    // Convert collision intervals into safe intervals for every cell in grid.getCells().

    return table;
}
```

### Main usage after refactor

```cpp
mapf::AStarSolver aStarSolver;
std::list<mapf::Cell*> path1 = aStarSolver.solve(grid, start1, end1);

mapf::AStarSippSolver sippSolver;
std::list<mapf::Cell*> path3 = sippSolver.solve(grid, start3, end3, otherAgentPaths);
```

## Validation Plan

1. Build the project with the updated compile command.
2. Run `./bin/main` and compare the printed SIPP path with the pre-refactor behavior.
3. Add a small compile-only check that includes each public header independently where practical.
4. Test at least these scenarios manually or with future unit tests:
   - A* returns an empty path when start is null.
   - A* returns an empty path when start or goal is blocked.
   - A* returns a path in a simple unobstructed grid.
   - SIPP returns an empty path when the start cell is unsafe at time 0.
   - SIPP avoids vertex conflicts with `otherAgentPaths`.
   - SIPP avoids edge swaps via `blockedEdgeArrivals`.

TODO: Introduce a test framework or simple executable tests so algorithm behavior can be validated automatically.

## Risks And Decisions

- Moving inline definitions out of headers will require updating the compile command; otherwise linking will fail.
- Hash functors used in `std::unordered_map` and `std::unordered_set` must remain visible to any translation unit using those containers.
- If `CompareAStarNode::operator()` remains non-const, some standard library implementations may still accept it, but using `const` is cleaner and safer.
- Returning only `safeIntervalsByCell` from `getSafeIntervalsByCell` would lose blocked edge information needed by `solve`; this plan returns `SafeIntervalTable` to keep the method coherent.
- `Grid::getCells()` exposes internal storage pointers indirectly. This matches the current design based on `Cell*`, but future changes could replace this with safer cell iteration APIs.

## Adjustments

No adjustments yet.
