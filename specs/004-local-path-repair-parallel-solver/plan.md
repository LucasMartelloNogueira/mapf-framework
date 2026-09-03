* Created at: 2026-09-02 21:49:35 -03
* Author: lucas
* Last updated at: 2026-09-02 21:57:23 -03
* AI model used: GPT-5 (Codex)

# Plan: Parallel Initial Planning With Local Path Repair

## Prompt

### User request

```text
leia o arquivo task.md e faça o que se pede
```

### Task definition from `task.md`

```text
leia o arquivo .github/ai-instructions.md e monte um plano para as seguinte funcionalidades:

descrição: Dado uma solução com um conjunto de caminhos de agentes que possuem conflitos, criar um mecanismo para reparo local de caminhos

Leia o arquivo docs/local_path_repair.md e entenda os seguintes cenários para reparo local. Monte o mecanismo seguindo as seguintes regras:

considere a seguinte nomeclatura:

* prefixo (do caminho): trecho inicial do caminho antes do conflito (considere ele sempre como válido)
* sufixo (do caminho): trecho do caminho após o conflito (ele pode ser válido ou não)
* caminho reconstruído: novo caminho que une o prefixo ao sufixo do caminho

Siga as mesmas diretrizes para o reparo local do caminho:
* assuma que todo o caminho antes do conflito (prefixo) é válido
* quando achar um conflito, divida o caminho em dois: prefixo e sufixo
* Use o SIPP para achar um novo caminho do prefixo até o sufixo, busque sempre achar o melhor caminho, buscando o caminho que chegue mais rápido no sufixo
* use uma janela adaptitva: se não achar um caminho do último vértice do prefixo até o primeiro vértice do sufixo, divida o prefixo em dois e começe o caminho do vértice do meio do prefixo. No pior caso, sempre que não achar um caminho, refaça todos o caminho do agente. Atualize a SafeIntervalTable e faça o SIPP da origem do agente até o destino considerando todos os conflitos do caminho desse agente.
* idea geral: em um caminho com k conflitos, ir sempre resolvendo todos os conflitos localmente, e ir tornando todos os trechos antes do conflitos como caminhos válidos, sem conflito. Fazer isso até resolver todos os confltios e chegar no destino 

considerando que o reparo local do agente é possível, veja o casos que podem acontecer em docs/local_path_repair.md e faça o seguinte em cada caso

* No cenário 1 (onde o agente chega no mesmo instante no sufixo que o no caminho anterior, que tinha conflito): considere todo o trecho do caminho do sufixo como válido, até chegar o próximo conflito. Caso não haja nenhum outro conflito no sufixo, considere todo o sufixo como válido

* No cenário 2: onde o agente chega em outro instante no sufixo, faça o seguinte em cada caso

  cenário 2.1: Fator 1 - sim / Fator 2 - sim
    fator 1: agente chegou no primeiro vértice do sufixo em algum instante antes de t
    fator 2: existe 1 ou mais agentes nos quais seus caminhos passam por algum vértice de B até G
    
    comportamento adotado: o agente deve esperar k timesteps até chegar no instante t. Depois deve seguir todos os vértice do sufixo até o próximo conflito ou até o final do caminho
    
 cenário 2.2: Fator 1 - sim / Fator 2 - não
    fator 1: agente chegou no primeiro vértice do sufixo em algum instante antes de t
    fator 2: nenhum outro agente passa, em seus caminhos, por algum vértice do caminho de B até G
    
    comportamento adotado: agente deve seguir todos os vértice do sufixo até o próximo conflito ou até o destino do agente
    
 cenário 2.3: Fator 1 - não / Fator 2 - sim
    fator 1: agente chegou no primeiro vértice do sufixo em algum instante depois de t
    fator 2: existe 1 ou mais agentes nos quais seus caminhos passam por algum vértice de B até G
    
    comportamento adotado: seguir o procedimento descrito no arquivo docs/path_sufix_repair_algorithm.md
    
 cenário 2.4: Fator 1 - não / Fator 2 - não
    fator 1: agente chegou no vértice B em algum instante depois de t
    fator 2: nenhum outro agente passa, em seus caminhos, por algum vértice do caminho de B até G
    
    comportamento adotado: agente deve seguir todos os vértice do sufixo até o próximo conflito ou até o destino do agente
    
    
 Independente do cenário, as seguintes informações globais da instância devem ser atualizadas:
 * os vértices e o custo do novo caminho do agente reparado devem ser atualizados
 * os safe intervals devem ser atualizados (safeIntervalsByCell e blockedEdgeArrivals)
 * vertex_agents (mapa no qual as keys são os vértices (Cell) e os valores são o conjunto de agentes que passam por aquele vértice): botar em todos os vértice que o agente passa, botar o id do agente no conjunto
 * goal_reservations[v] (mapa no qual as keys são os vértices de chegada dos agentes (Cell) e os valores é o timestep no qual o agente chegou e permaneceu no seu vértice de destino)
 
 
 Para fazer tudo isso, faça também o seguintes itens:
 
 * em src/mapf/pathfinding/a_star_sipp.cpp criar um método chamado solve que recebe como parâmetros um grid, um vértice de origem, um vértice de destino, SafeIntervalTable e um tempo t: esse metódo deve fazer a busca SIPP da origem até o destino usando a SafeIntervalTable recebida e assumir que o instante t é o istante de começo da busca
 * em include/mapf/solvers e src/mapf/solvers: criar um arquivo chamado local_path_repair_parallel_solver.hpp e c que define uma classe de solver que dado uma instance e um número de threads, cria um um thread pool e acha todos os caminhos dos agentes (usando AStarSolver) de forma paralela, independente dos conflitos, e retorna os caminhos e os conflitos deles (usando o método getColission de src/mapf/utils.cpp). Depois de achado os caminhos, atualizar a SafeIntervalTable, vertex_agents e goal_reservations. Depois, para cada agente, ver os conflitos que seus caminhos possuem e consertar eles localmente.
 
 
```

## Objective

Add a solver that first computes every agent's unconstrained shortest path concurrently and then repairs the resulting conflicts sequentially. Each repair must preserve the already validated prefix, reconnect it to the old suffix with earliest-arrival SIPP, expand the repair window toward the source when necessary, and fall back to a full SIPP replan. The solver must finish with synchronized path costs, safe intervals, blocked edge arrivals, vertex-to-agent membership, and permanent goal reservations.

Only initial unconstrained A* searches are parallel. Local repairs are deliberately sequential so every SIPP call sees a frozen snapshot of every other path.

## Context And Existing Codebase Analysis

### MAPF semantics

`.github/mapf.md` defines a discrete-time, four-connected MAPF model with move and wait actions. The project uses stay-at-target semantics: after reaching its goal, an agent occupies that cell indefinitely. Current code detects simultaneous vertex occupation and opposite-direction edge swaps.

The source context file is `.github/mapf.md`; there is no root-level `mapf.md`. This plan uses the existing file in `.github`, which is also the location implied by the repository instructions and prior plans.

### Current planning APIs

- `AStarSolver::solve(Grid&, Cell*, Cell*)` returns an unconstrained shortest path as `std::list<Cell*>`. The list index is the absolute timestep when planning starts at time zero.
- `AStarSippSolver::solve(Grid&, Cell*, Cell*, const std::vector<std::list<Cell*>>&)` builds a reservation table internally and starts at time zero.
- `AStarSippSolver::getSafeIntervalsByCell` actually returns the whole `SafeIntervalTable`, including `safeIntervalsByCell` and reverse-edge arrival reservations in `blockedEdgeArrivals`.
- SIPP already stores one search state per `(Cell*, intervalIndex)`, uses Manhattan distance, reconstructs explicit waits, and chooses an earliest-arrival goal under its current A* ordering.
- The requested overload that accepts an existing `SafeIntervalTable` and a nonzero start time does not exist. A TODO for it is already present at the end of `src/mapf/pathfinding/a_star_sipp.cpp`.

The new overload must use absolute time for `SippNode::time`, `g`, safe-interval lookup, edge checks, and reconstruction. Its returned list still represents only the path segment: if it starts at absolute time `t`, its first list element is the start cell at `t`, and arrival time is `t + path.size() - 1`.

### Current collision API

`getCollision` is the actual function name; `getColission` in the task is a spelling error. It returns `SolutionConflicts`, whose records contain cells and times but no agent IDs or path indexes.

The current implementation is path-order dependent for stay-at-target conflicts. It only registers an agent's permanent goal after scanning that path, so it can miss a long path entering the goal of a shorter path that appears later in the input vector. Local repair cannot rely on incomplete conflict data.

The implementation should become time-major and pairwise:

1. Convert each list to an indexable vector once.
2. Evaluate timesteps from zero through the current makespan.
3. Define `positionAt(path, t)` as `path[t]` while `t < path.size()` and `path.back()` afterward.
4. Report every pairwise vertex conflict and every opposite-direction edge swap independent of vector order.

The public conflict structs do not need to change. A private solver helper can map a returned cell/time or edge/time record back to involved path indexes by inspecting `positionAt`. A virtual stay-at-goal occupant is not selected for repair when another agent explicitly enters that cell; the moving agent is selected. A shared start at `t = 0` and duplicate permanent goals are terminal failures because the fixed endpoints cannot be repaired away.

### Current solution and ownership model

`Instance` owns `Agent` values and its `Grid`; returned `Cell*` values point into the grid's stable `std::vector<Cell>` storage. The new solver should retain `const Instance&`, just as `PriorityPlanningSolver` does, and must not mutate the problem definition. Reservation tables and path indexes are derived solution state and belong to the new solver/result.

`Result` currently contains only aggregate metrics and no paths or conflicts. A dedicated return type is therefore needed for the new solver.

### Build and tests

The project uses CMake and C++20, has no external dependencies, and currently defines no CTest tests. The baseline `cmake --build build -j2` succeeds. New concurrency code must be linked portably through CMake's `Threads::Threads` target.

## Documentation And Libraries Researched

No third-party runtime library is required.

- The original SIPP paper defines a state as a configuration plus a safe interval, keeps the earliest reachable time for that state, generates wait-and-move successors, and establishes time-minimal goal arrival. This matches the task's requirement to reconnect to the suffix as early as possible: https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf
- The C++ working draft specifies `std::jthread`, condition variables, futures, and `std::packaged_task`. These are sufficient for a small bounded worker pool whose tasks return paths and propagate failures through futures: https://eel.is/c++draft/thread and https://eel.is/c++draft/futures.task
- CMake's `FindThreads` module provides the portable `Threads::Threads` imported target: https://cmake.org/cmake/help/latest/module/FindThreads.html
- `BS::thread_pool` was evaluated as a maintained single-header alternative: https://github.com/bshoshany/thread-pool. It is not recommended here because the solver needs only fixed-size task submission during one phase; the standard library can provide that without vendoring and maintaining another dependency.

Use these C++20 standard-library facilities:

- `<thread>` / `std::jthread` for workers that automatically join.
- `<mutex>`, `<condition_variable>`, and `<queue>` for a protected task queue.
- `<future>` and `std::packaged_task` for indexed A* results and exception propagation.
- `<unordered_map>` and `<unordered_set>` for reservation indexes.
- `<vector>`, `<list>`, `<optional>`, and `<algorithm>` for path storage and repair selection.
- `<chrono>` and `<cmath>` for the result duration and injustice metric.

## Proposed Public Interfaces

### Time-offset SIPP

Add the exact requested overload, passing the table by const reference because the search does not own or modify it:

```cpp
namespace mapf {

class AStarSippSolver {
public:
    std::list<Cell*> solve(
        Grid& grid,
        Cell* start,
        Cell* goal,
        const SafeIntervalTable& safeIntervalTable,
        int startTime
    );

    // Existing overloads remain available.
};

}
```

Move the shared search into one private implementation so the current path-based overload builds a table and delegates with `startTime = 0`. Add an internal goal policy used only when replanning to an agent's real destination: local bridge endpoints may be left after arrival, while a real MAPF goal is valid only when its selected safe interval extends to the table's infinity sentinel.

Expose the infinity sentinel from `safe_interval_table.hpp` instead of duplicating the current source-local `INF_TIME` constant:

```cpp
inline constexpr int SAFE_INTERVAL_INFINITY = std::numeric_limits<int>::max() / 4;
```

### Reservation state

Keep the requested indexes together so they can only be committed as one consistent snapshot:

```cpp
namespace mapf {

using VertexAgents = std::unordered_map<Cell*, std::unordered_set<int>>;
using GoalReservations = std::unordered_map<Cell*, int>;

struct PathReservationState {
    SafeIntervalTable safeIntervalTable;
    VertexAgents vertex_agents;
    GoalReservations goal_reservations;
};

}
```

`goal_reservations[cell]` is `path.size() - 1`. Duplicate goal cells are rejected as unsolvable under stay-at-target semantics instead of silently overwriting this map.

### Solver result and solver

```cpp
namespace mapf {

struct LocalPathRepairResult {
    Result metrics;
    std::vector<std::list<Cell*>> paths;
    std::vector<int> pathCosts;
    SolutionConflicts initialConflicts;
    SolutionConflicts remainingConflicts;
    PathReservationState reservations;
};

class LocalPathRepairParallelSolver {
public:
    LocalPathRepairParallelSolver(const Instance& instance, std::size_t numberOfThreads);

    LocalPathRepairResult solve();

private:
    const Instance& instance;
    std::size_t numberOfThreads;
};

}
```

Semantics:

- `metrics.success` is true only if every initial A* succeeds, every required repair succeeds, and `remainingConflicts` is empty.
- `paths[i]` and `pathCosts[i]` correspond to `instance.getAgents()[i]`.
- `initialConflicts` is the conflict set immediately after the parallel A* phase.
- `remainingConflicts` is the final conflict set. It must be empty on success and preserves diagnostic information on failure.
- `reservations` always describes the returned `paths`, including on a repair failure after at least one successful commit.
- `sumOfCosts`, `makespan`, and every repaired agent's `pathCosts[i]` are refreshed after each accepted repair. Injustice uses the standard deviation of `finalCost[i] - initialOptimalCost[i]`; the initial A* paths already provide the unconstrained optimum needed for this calculation.
- `numberOfThreads == 0` is rejected with `std::invalid_argument`. An empty agent set is a valid empty solution.

## Reservation Rebuild Strategy

Do not mutate `safeIntervalsByCell` or `blockedEdgeArrivals` in place. Those structures do not retain enough per-agent provenance to safely delete an old repaired path, especially when several agents reserve the same cell/time or edge/time.

Use one deterministic builder:

```cpp
PathReservationState buildReservationState(
    Grid& grid,
    const std::vector<Agent>& agents,
    const std::vector<std::list<Cell*>>& paths,
    std::optional<std::size_t> excludedPathIndex = std::nullopt
);
```

For every included path, the builder must:

1. Add every finite path cell to `vertex_agents[cell]` using the real `Agent::id`. Repeated visits by one agent still create one set entry.
2. Reserve each occupied `(cell, timestep)` in the data used to derive safe intervals.
3. Insert the reverse directed edge and arrival timestep in `blockedEdgeArrivals` so opposite-direction swaps are rejected.
4. Reserve the final cell from `path.size() - 1` through `SAFE_INTERVAL_INFINITY`.
5. Store the final arrival in `goal_reservations`.
6. Merge adjacent collision times and complement them into sorted, non-overlapping safe intervals for every grid cell.

Two snapshots are used:

- The committed global snapshot includes all current paths and is returned to the caller.
- Before repairing agent `i`, a temporary snapshot excludes path `i`. This prevents the agent's obsolete path from blocking its own SIPP search while all other paths remain frozen.

After a candidate path passes validation, replace only `paths[i]`, update its cost, and rebuild the complete committed snapshot. Rebuilding is intentionally favored over incremental removal because it makes stale cells, stale edges, and changed goal times impossible.

## Initial Parallel Planning

Create a small thread pool private to `local_path_repair_parallel_solver.cpp`:

```cpp
class ThreadPool {
public:
    explicit ThreadPool(std::size_t workerCount);

    template<class Function>
    auto submit(Function&& function)
        -> std::future<std::invoke_result_t<Function>>;

private:
    std::vector<std::jthread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex mutex;
    std::condition_variable ready;
    bool stopping = false;
};
```

Planning flow:

1. Allocate the final path vector at agent count before starting workers.
2. Create `min(numberOfThreads, max(agentCount, 1))` workers.
3. Submit one task per agent. Each task creates its own stateless `AStarSolver` and reads the shared grid only.
4. Store futures in instance-agent order and collect them in that same order. Parallel completion order must never change the path-to-agent mapping.
5. If any initial path is empty, stop before repair and return failure with every completed diagnostic path preserved.
6. Destroy/join the pool before constructing reservation state or beginning repair. No worker may observe mutable path/reservation state.
7. Call `getCollision(paths)`, store `initialConflicts`, and build the initial committed reservation state.

The `Grid` object must remain immutable during this phase. Add const-qualified read APIs or document and enforce that the existing non-const functions used by A* do not write. No task shares an `AStarSolver` instance or mutable search container.

## Conflict Selection And Path Splitting

Process agents in `Instance::getAgents()` order. For each agent, repeatedly recompute conflicts and select its earliest explicit conflict. Previously repaired paths and all not-yet-repaired paths remain frozen during this attempt.

Conflict ownership rules:

- For a vertex conflict at time `tc`, an agent is repairable when its finite path explicitly contains that timestep and occupies the reported cell.
- An agent that is present only because it has already reached its permanent goal is not selected; the later moving agent must route around that goal.
- For an edge conflict arriving at time `tc`, an agent is involved when its transition from `tc - 1` to `tc` matches either direction of the reported edge.
- If several agents are explicitly involved, instance order breaks the tie.
- A vertex conflict at time zero is an immediate failure because starts are fixed.
- Two agents with the same goal are an immediate failure under permanent goal occupation.

Split indexes according to conflict type:

| Conflict | Valid prefix ends at | Old suffix starts at | Reason |
| --- | ---: | ---: | --- |
| Vertex at `tc` | `tc - 1` | `tc + 1` | The conflicted vertex itself must be removed from the local path. |
| Edge swap arriving at `tc` | `tc - 1` | `tc` | The arrival vertex may remain, but the conflicting transition must be replaced. |

If either side does not exist, skip directly to full-path replanning. Prefix and suffix endpoints are included in their respective segments. When splicing, remove duplicate anchor cells so each list position still represents exactly one timestep.

## Adaptive Repair Window

For a conflict with prefix endpoint index `a` and suffix start index `b`:

1. Freeze the path prefix through `a` and the old suffix from `b` onward.
2. Build the temporary reservation table from every path except the active agent.
3. Run time-offset SIPP from `path[a]` at absolute time `a` to `path[b]`.
4. SIPP must minimize absolute arrival time at `path[b]`. Compute it as `a + bridge.size() - 1`.
5. If no bridge is found or the scenario-specific suffix handling is invalid, expand left using `a = floor(a / 2)`. This retains `path[0..a]` and discards more of the formerly valid prefix.
6. Repeat halving until anchor zero has been attempted. Track attempted anchors to avoid repeating zero.
7. If no local window succeeds, run SIPP from the true origin at time zero to the true goal using all other paths as reservations and requiring a safe interval that remains open forever at the goal.
8. If full replanning fails, return failure. If it succeeds, it replaces the entire active path and resolves all of that agent's current conflicts in one step.

Illustrative control flow:

```cpp
while (auto conflict = findEarliestExplicitConflict(agentIndex, paths, getCollision(paths))) {
    std::optional<std::list<Cell*>> repaired;

    for (std::size_t anchor : adaptiveAnchors(conflict->prefixEndIndex)) {
        auto bridge = sipp.solve(
            grid,
            pathAt(anchor),
            pathAt(conflict->suffixStartIndex),
            reservationsWithoutAgent.safeIntervalTable,
            static_cast<int>(anchor)
        );

        repaired = combineAccordingToScenario(anchor, bridge, oldSuffix, ...);
        if (repaired && candidateIsValidAgainstFrozenPaths(*repaired, agentIndex, paths)) {
            break;
        }
    }

    if (!repaired) {
        repaired = replanWholePathWithPermanentGoal(...);
    }

    if (!repaired) {
        return failureResult(...);
    }

    commitPathAndRebuildReservations(agentIndex, *repaired);
}
```

After each commit, require measurable progress: the earliest explicit conflict for that agent must move later, disappear, or full replanning must produce a conflict-free path. Keep fingerprints of `(path cells, earliest conflict)` during one agent's repair so a repeated candidate becomes failure/fallback instead of an infinite loop.

## Suffix Scenario Rules

Let `B` be the first old suffix cell, `G` the agent goal, `originalBTime` its index in the old path, and `newBTime` the bridge arrival time. `hasPotentialSuffixConflict` is true when any cell in old suffix `B..G` has an agent ID other than the active agent in `vertex_agents`. Checking for another ID is more robust than a raw set-size check and is equivalent to the document's `size >= 2` rule when the active path is indexed correctly.

| Scenario | Condition | Required behavior |
| --- | --- | --- |
| 1 | `newBTime == originalBTime` | Append the old suffix with its original timing. Treat it as valid through the next conflict; if none exists, accept through `G`. |
| 2.1 | `newBTime < originalBTime` and potential conflict exists | Repeat `B` exactly `originalBTime - newBTime` times, then append the old suffix at its original absolute times. The wait is valid only if `B` remains in one safe interval throughout the wait. If not, reject this window and expand/fallback. |
| 2.2 | `newBTime < originalBTime` and no potential conflict exists | Append the entire suffix immediately, shifted earlier. No other frozen agent uses its vertices, so no vertex or swap conflict can be introduced there. |
| 2.3 | `newBTime > originalBTime` and potential conflict exists | Repair the suffix transition by transition using `docs/path_sufix_repair_algorithm.md`. |
| 2.4 | `newBTime > originalBTime` and no potential conflict exists | Append the entire suffix shifted later. The final cost increases by the delay. |

For scenarios 1 and 2.1, it is acceptable to splice the whole suffix and then find the next conflict immediately; the validated-prefix marker advances only to the cell before that conflict. This produces the same semantics without maintaining a second partial-path representation.

Every combined candidate must be checked against the frozen other paths before commit. This final check covers goal occupation and protects against an incorrect potential-conflict classification.

## Scenario 2.3: Suffix Repair

Start at `B` at `newBTime` and inspect each original suffix transition `(current, next)`. A cell has potential conflict when `vertex_agents[cell]` contains an ID other than the active agent. Use the reservation table that excludes the active old path.

| Current cell | Next cell | Operation |
| --- | --- | --- |
| No potential conflict | No potential conflict | Append the original one-timestep transition. |
| Potential conflict | No potential conflict | The current arrival was already validated; append the transition after checking the reverse-edge arrival reservation. |
| No potential conflict | Potential conflict | Find the earliest safe arrival at `next`. Wait at `current` only while its current safe interval remains valid, and reject blocked edge arrivals. Append explicit wait cells followed by `next`. |
| Potential conflict | Potential conflict | Run time-offset SIPP from `current` at its actual time to `next`; append the returned mini-path without duplicating `current`. Return failure if no mini-path exists. |

Additional invariants:

- Every direct transition must be either a wait or a four-connected grid edge.
- `blockedEdgeArrivals` is checked even in cases where vertex membership suggests no potential collision.
- A wait may not cross the end of the current cell's safe interval.
- If `next` is the agent's real goal, acceptance requires its safe interval to extend to `SAFE_INTERVAL_INFINITY`.
- A SIPP mini-path may detour through cells outside the original suffix; those cells are already validated by SIPP and are included when rebuilding `vertex_agents`.
- Any failure returns control to the adaptive-window loop. It does not partially mutate the committed path or reservation state.

## Files To Change Or Create During Implementation

### `include/mapf/pathfinding/a_star_sipp.hpp`

- Add the requested five-argument `solve` overload.
- Pass `SafeIntervalTable` by const reference.
- Keep the existing path-based overload for compatibility.
- Declare a private shared search implementation or a dedicated permanent-goal helper.

### `src/mapf/pathfinding/a_star_sipp.cpp`

- Extract the existing search body so both public overloads share it.
- Initialize the start state at `startTime`, not zero.
- Reject negative start times and a start cell that is not safe at that exact time.
- Use absolute arrival values in `bestG`, `SippNode`, interval checks, and blocked-edge checks.
- Preserve explicit waits during reconstruction.
- Support transient bridge goals and permanent real goals without duplicating the search.
- Remove the existing TODO once the overload is implemented.

### `include/mapf/pathfinding/sipp/safe_interval_table.hpp`

- Expose the common infinity sentinel.
- Add `VertexAgents`, `GoalReservations`, and `PathReservationState`, or place those solver-specific aliases in the new solver header if keeping SIPP primitives minimal is preferred.

Recommended placement: keep only `SAFE_INTERVAL_INFINITY` in the SIPP header and put solution-derived maps in the solver header.

### `include/mapf/solvers/local_path_repair_parallel_solver.hpp`

- Create `LocalPathRepairResult` and `LocalPathRepairParallelSolver`.
- Document index alignment, failure behavior, thread-count validation, and reservation ownership.
- Keep thread-pool implementation details out of the public header.

### `src/mapf/solvers/local_path_repair_parallel_solver.cpp`

- Create the bounded thread pool.
- Run indexed parallel A* tasks.
- Build and atomically rebuild all reservation indexes.
- Map location/time conflicts back to affected path indexes.
- Implement conflict splitting, adaptive anchors, path splicing, all four suffix scenarios, suffix transition repair, full SIPP fallback, progress detection, and metrics.
- Use local candidate objects so a failed attempt cannot corrupt committed state.

The task's requested source extension `c` should be implemented as `.cpp`; the class and the entire repository use C++20, and CMake currently compiles solver implementations as C++ sources.

### `include/mapf/utils.hpp`

- Keep `getCollision` as the public spelling.
- No signature change is required.
- Optionally add a small documented `positionAt` helper only if it is useful outside the new solver; otherwise keep it private.

### `src/mapf/utils.cpp`

- Make `getCollision` time-major and path-order independent.
- Apply stay-at-target position semantics for all agents through makespan.
- Preserve vertex and opposite-edge-swap conflict types.
- Ensure one pairwise conflict is emitted once per cell/edge/timestep.

### `CMakeLists.txt`

- Add `src/mapf/solvers/local_path_repair_parallel_solver.cpp` to `mapf`.
- Add `find_package(Threads REQUIRED)` and link `Threads::Threads` to `mapf`.
- Enable CTest and add focused test executables.

### `tests/pathfinding/a_star_sipp_test.cpp`

- Test nonzero start time, blocked start time, explicit waits, blocked reverse-edge arrival, earliest arrival, transient bridge goals, and permanent final goals.
- Verify the existing path-based overload still works.

### `tests/utils_collision_test.cpp`

- Test order-independent stay-at-goal detection.
- Test vertex conflicts, edge swaps, path pairs, and no-conflict input.
- Test unequal path lengths and three-agent conflicts.

### `tests/solvers/local_path_repair_parallel_solver_test.cpp`

- Cover the initial parallel path-to-agent ordering and thread counts of one, fewer than agents, and greater than agents.
- Cover Scenario 1 and Scenarios 2.1 through 2.4.
- Exercise all four suffix-transition cases from `docs/path_sufix_repair_algorithm.md`.
- Cover a last-prefix-vertex failure followed by midpoint success.
- Cover a repair that reaches the origin and a full-path fallback.
- Cover one agent with multiple sequential conflicts.
- Cover vertex, edge-swap, and stay-at-target conflicts.
- Assert that old `vertex_agents` entries and edge reservations disappear after a changed path, new entries appear, and goal arrival/costs change correctly.
- Assert deterministic results across repeated runs despite parallel initial planning.
- Cover zero threads, unreachable initial path, shared start, duplicate goals, failed repair, empty instance, and start-equals-goal.
- Require `validateSolution(result.paths)` and empty `remainingConflicts` on every success.

### `docs/local_path_repair_parallel_solver.md`

- Document the solver API, time/index conventions, reservation semantics, adaptive-window sequence, scenario table, failure behavior, and complexity.
- Reference the two existing design documents without overwriting them.

## Implementation Sequence

1. Correct `getCollision` so the repair loop has complete, path-order-independent conflict information.
2. Expose the safe-interval infinity sentinel and implement the time-offset SIPP overload with focused tests.
3. Define the solver result and reservation-state contracts.
4. Implement and test full reservation rebuilding, including exclusion of the active agent.
5. Implement the bounded initial A* thread pool and deterministic future collection.
6. Implement conflict ownership, vertex/edge split indexes, path splice helpers, and adaptive anchors.
7. Implement Scenario 1, 2.1, 2.2, and 2.4.
8. Implement Scenario 2.3's four transition cases.
9. Add full-path permanent-goal SIPP fallback and progress-cycle detection.
10. Commit path/reservation changes atomically and recalculate costs and metrics.
11. Validate the final paths with `getCollision` and `validateSolution`.
12. Add CMake test targets, run the full suite, and document the completed solver.

## Complexity Expectations

Let `A` be the number of agents, `L` the total finite path length, `V` grid cells, and `R` accepted repair commits.

- Initial planning performs `A` independent A* searches with at most `numberOfThreads` active tasks.
- A reservation rebuild is `O(V + L log L)` with the current per-cell interval merge approach; it occurs initially and after each accepted repair.
- Conflict detection is `O(A^2 * makespan)` in the proposed pairwise time-major implementation. This is acceptable for correctness-first local repair and can later be reduced with occupancy maps that retain participant indexes.
- Local SIPP retains the existing state bound of cell-safe-interval pairs. Adaptive windowing can invoke it `O(log prefixLength)` times before the one full-path fallback.

## Acceptance Criteria

- The new solver accepts an `Instance` and a positive thread count.
- Initial unconstrained A* paths are computed through a bounded thread pool and returned in instance-agent order.
- Local repair itself never runs concurrently.
- `getCollision` detects vertex, edge-swap, and stay-at-target conflicts independent of input path order.
- The requested SIPP overload accepts an existing table and an absolute start time and returns an earliest-arrival segment with explicit waits.
- Each SIPP repair excludes the active agent's obsolete reservations and includes every other current path.
- Vertex conflicts remove the conflicted vertex from the local bridge; edge conflicts remove the conflicting transition.
- Failed local bridges expand the prefix anchor toward the source and eventually trigger a full origin-to-goal SIPP replan.
- Scenario 1 and Scenarios 2.1 through 2.4 behave exactly as specified, including the required wait in 2.1.
- Scenario 2.3 implements all four current/next potential-conflict combinations and checks blocked edge arrivals.
- An agent with multiple conflicts is repaired repeatedly until none remain or a repair proves impossible.
- After every accepted repair, paths, per-agent costs, aggregate metrics, `safeIntervalsByCell`, `blockedEdgeArrivals`, `vertex_agents`, and `goal_reservations` all describe the same committed solution.
- Replaced-path reservations do not remain in any derived structure.
- Real goals are reserved indefinitely and a successful path ends in a permanently safe interval.
- A successful result has no remaining conflicts and passes `validateSolution`.
- Invalid thread counts and endpoint conflicts that cannot be repaired return or throw according to the documented contract without partial state corruption.
- `cmake --build build` and all CTest targets succeed.

## Risks And Explicit Decisions

- Rebuilding reservation state costs more than incremental updates but avoids incorrect deletion when reservations overlap. Optimize only after profiling.
- Sequential repair is prioritized by instance order and is not globally optimal across all agents. Each individual SIPP bridge is earliest-arrival for its chosen endpoints and frozen reservations.
- A Scenario 2.1 bridge that arrives early but cannot safely wait at `B` is not accepted. The window expands and ultimately uses full SIPP rather than violating the mandated wait behavior.
- The project currently treats opposite-direction swaps as edge conflicts. Same-direction traversal already coincides at the arrival vertex in this unit-time grid model and is reported as a vertex conflict.
- The original documents assume local repair is possible. The implementation still needs explicit failure results for unreachable paths, fixed shared starts, duplicate permanent goals, or exhausted fallback.

## TODO

TODO: After correctness tests exist, profile full reservation rebuilding and time-major pairwise collision detection before designing incremental counted reservations.

TODO: Consider extracting a reusable project-wide thread pool only if another feature needs the same concurrency abstraction; keep this first implementation private to the solver.

TODO: Decide in a later API cleanup whether conflict records should directly carry participating agent indexes. The private mapping in this plan avoids changing the public collision types now.

## Adjustments

No adjustments have been requested yet.
