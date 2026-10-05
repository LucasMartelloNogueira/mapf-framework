Created: 2026-10-02T13:22:21-03:00

Author: Lucas Martello Nogueira (local user: lucas; repository Git identity)

Last updated: 2026-10-02T13:23:31-03:00

AI model: GPT-6 (Codex)

# Plan: FullPathRepairIterativeSolver

## Objective and scope

Add `mapf::FullPathRepairIterativeSolver`: compute every agent's initial path sequentially with `AStarSolver`, detect conflicts with `getCollision`, and repair one conflict at a time by replanning the complete paths of all its participants with `AStarSippSolver`, in ascending `Agent::id` order.

Build the reservation state once after the initial searches. For each selected conflict, remove **all** participants' old reservations before the first SIPP search. Then search for one participant and immediately insert its new reservations before searching for the next participant. All searches start at the original start cell at time zero and finish at the original goal with permanent occupancy.

This deliverable is the requested implementation plan. The sections below describe future source changes, tests, and experiment integration.

## References and existing architecture

Follow [repository instructions](../../.github/ai-instructions.md) and [MAPF conventions](../../.github/mapf.md). Reuse the contracts established by:

- [Feature 005 plan](../005-cli-results-and-iterative-local-repair/plan.md) and [specification](../005-cli-results-and-iterative-local-repair/spec.md): sequential initial planning, aligned result vectors, metrics, CLI, and artifacts.
- [Feature 007 plan](../007-collision-index-and-local-repair-strategies/plan.md) and [specification](../007-collision-index-and-local-repair-strategies/spec.md): conflict events, identities, input validation, and parked destination owners.
- [Feature 008 plan](../008-incremental-path-reservations/plan.md), [specification](../008-incremental-path-reservations/spec.md), and [reservation documentation](../../docs/incremental_path_reservatons.md): incremental removal/insertion, disposable state copies, and permanent-goal SIPP searches.

The inspected implementation already provides these components:

| Component | Relevant behavior and reuse |
| --- | --- |
| `src/mapf/solvers/local_path_repair_iterative_solver.cpp` | Sequential A* initialization, preserving instance order in path vectors. Follow this pattern for the new solver. |
| `include/mapf/solvers/local_path_repair_solver.hpp` | `LocalPathRepairResult` already carries metrics, initial/final paths, costs, initial/remaining conflicts, and reservations. Reuse it as the result container. |
| `src/mapf/utils.cpp` | `getCollision(paths)` returns global vertex/edge events and `byAgent`. Participant values are **path indexes**, not agent IDs. |
| `src/mapf/solvers/local_path_repair_solver_common.hpp/.cpp` | `buildReservationState`, `repairSafeIntervalTable`, and `updateReservationState` are available in `local_path_repair_detail`. Reservation ownership uses real agent IDs. |
| `include/mapf/pathfinding/a_star_sipp.hpp` | The existing six-argument overload accepts a supplied table, absolute start time, and `GoalOccupation::Permanent`. The five-argument table overload is transient; the path-vector overload builds a table internally. |
| `src/mapf/solvers/local_path_repair_solver_common.cpp` | `updateResultMetrics` currently has anonymous-namespace linkage. Expose this existing implementation through the internal common header to share metric calculation. |
| `src/mapf/core/instance.cpp` | Both constructors reject repeated starts or repeated goals before solving; IDs are unique and may be noncontiguous or negative. |
| `src/main.cpp`, `src/mapf/experiments/experiment_utils.cpp` | Solver dispatch, metadata validation, and failed-run conflict artifacts currently recognize only the three existing solvers. Extend these integration points. |
| `CMakeLists.txt`, `tests/test_support.hpp` | C++20 library, CMake/CTest, and lightweight test executables already exist. |

The current local engine repairs one active agent and tries local windows before full fallback. The new solver needs its own conflict loop because it removes every participant and always replans complete paths. Reuse its reservation and metric helpers without invoking `repairInitialPaths` or changing local-repair scheduling.

## Libraries and documentation researched

Keep C++20 and the existing project dependencies. No additional runtime or test library is needed.

| Facility or reference | Decision |
| --- | --- |
| Existing `AStarSolver` and `AStarSippSolver` | Reuse both implementations. The [original SIPP paper](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf) describes search over configurations and safe intervals; this repository already supplies the required search and explicit waits. |
| Standard containers, `std::sort`, `std::tuple`, `std::chrono` | Keep paths in vectors/lists, deduplicate participants, impose deterministic ID/event ordering, and measure total solver duration. Supply an explicit comparator as described in the [C++ sorting specification](https://eel.is/c++draft/alg.sort). |
| CMake/CTest | Register the new source and test executable with the existing build. Use the existing [`add_test(NAME ... COMMAND ...)` form](https://cmake.org/cmake/help/latest/command/add_test.html). |
| Boost ICL, considered | Its [interval containers](https://www.boost.org/doc/libs/latest/libs/icl/doc/html/index.html) would add a dependency without replacing the required reservation-owner and reverse-edge bookkeeping. Keep the existing interval representation and mutation helpers. |

## Proposed API and default decisions

The user specifies the search engines, update lifecycle, and participant priority. The following choices complete the first version without adding configuration:

1. Return the existing `LocalPathRepairResult` type; its data already describes this algorithm.
2. Stop on the first unsuccessful conflict-group repair, returning the last committed snapshot with `metrics.success == false`. A failed prioritized attempt does not establish that the MAPF instance is unsolvable.
3. Commit a conflict group atomically: if a later participant cannot find a path, discard the entire group's temporary changes.
4. Use fixed ascending agent IDs within a conflict. Choose between conflicts deterministically as described below.
5. Make the solver available to existing experiments through `-solver FullPathRepairIterativeSolver`, with no solver-specific flags in this version.

```cpp
// include/mapf/solvers/full_path_repair_iterative_solver.hpp
#pragma once

#include "mapf/core/instance.hpp"
#include "mapf/solvers/local_path_repair_solver.hpp"

namespace mapf {
    class FullPathRepairIterativeSolver {
        private:
            const Instance& instance;

        public:
            explicit FullPathRepairIterativeSolver(const Instance& instance);
            LocalPathRepairResult solve();
    };
}
```

Keep all result vectors in `instance.getAgents()` order. Sort a separate collection of participant indexes, comparing `agents[index].id`; never reorder the agents or interpret an ID as a vector position. Cell pointers remain non-owning references to the instance's grid.

## Implementation sequence

### 1. Initialize paths and reservations once

Start the duration clock before the first A* search. Allocate one path slot per agent and call `AStarSolver::solve` sequentially for every agent, ignoring other agents. Attempt all initial searches even if an earlier one returns an empty path.

Validate nonempty paths against their original endpoints, free cells, legal cardinal moves/waits, and the existing maximum length `SAFE_INTERVAL_INFINITY` before reservation construction. Empty slots represent search failure and remain aligned. The existing `Instance` constructors handle instance validity before this stage.

```cpp
const auto startedAt = std::chrono::steady_clock::now();
const auto& agents = instance.getAgents();
Grid& grid = const_cast<Grid&>(instance.getGrid()); // Existing solver convention.

LocalPathRepairResult result;
result.initialPaths.resize(agents.size());
for (std::size_t i = 0; i < agents.size(); ++i) {
    AStarSolver astar;
    result.initialPaths[i] = astar.solve(
        grid,
        grid.getCellPtr(agents[i].startPosition.x, agents[i].startPosition.y),
        grid.getCellPtr(agents[i].goalPosition.x, agents[i].goalPosition.y));
}
// Validate all nonempty paths here before building reservations.
result.paths = result.initialPaths;
result.initialConflicts = getCollision(result.initialPaths);
result.remainingConflicts = result.initialConflicts;
result.reservations = local_path_repair_detail::buildReservationState(
    grid, agents, result.paths);
```

This is the only full reservation build in a solver invocation. If any required initial path is missing, return unsuccessful metrics with this consistent initial snapshot and perform no repair. An empty instance succeeds with empty paths and zero costs. When initial paths are already conflict-free, return them without invoking SIPP.

### 2. Select one current global conflict and all participants

Select from `remainingConflicts.vertexEvents` and `remainingConflicts.edgeEvents`:

- A vertex event contributes its entire `participants` set.
- A swap event contributes the union of `forward` and `reverse`; deduplicate the resulting indexes.
- Include destination owners parked beyond their finite path length. These owners appear in global vertex events but can be absent from `byAgent[owner]` at that time. Therefore, `byAgent` alone must not determine membership or repair success.
- A group contains all participants of the selected event, including events with three or more agents. It is not a transitive connected component of all current conflicts.

For deterministic event selection, compare `(minimum participant Agent::id, time, kind, x1, y1, x2, y2)`, with vertex kind before edge kind, the vertex cell repeated for both endpoints, and the detector's canonical endpoints for edges. This is an explicit tie-breaking choice for the plan. Never depend on hash-map iteration or pointer-address ordering.

Store a value copy of the selected event key and its participant indexes. Sort the indexes by real agent ID:

```cpp
std::sort(participants.begin(), participants.end(),
    [&](std::size_t first, std::size_t second) {
        return agents[first].id < agents[second].id;
    });
```

For example, instance slots containing IDs `[42, 7]` must be repaired in slot order `[1, 0]`. Each ID remains associated with its original path slot and scenario bucket.

### 3. Remove the entire group, then replan and insert sequentially

Create one disposable reservation-state copy per selected conflict, following feature 008's exception-safety contract. Copy the candidate paths once as well. Copying the existing state does not reconstruct reservations from paths, but it still has a memory/time cost.

The two loops below must remain separate. `participants` contains every selected event participant, already ordered by ID. `validCompletePath` is a small internal structural validator to implement; it checks nonempty paths, original endpoints, free cells, cardinal moves/waits, and representable times.

```cpp
auto candidatePaths = result.paths;
auto candidateReservations = result.reservations;

// Finish ALL removals before any SIPP call.
for (std::size_t index : participants) {
    local_path_repair_detail::repairSafeIntervalTable(
        candidateReservations, result.paths[index], agents[index].id);
}

AStarSippSolver sipp;
bool groupSucceeded = true;
for (std::size_t index : participants) {
    Cell* start = grid.getCellPtr(
        agents[index].startPosition.x, agents[index].startPosition.y);
    Cell* goal = grid.getCellPtr(
        agents[index].goalPosition.x, agents[index].goalPosition.y);
    auto replacement = sipp.solve(
        grid, start, goal, candidateReservations.safeIntervalTable,
        0, AStarSippSolver::GoalOccupation::Permanent);

    if (!validCompletePath(replacement, start, goal)) {
        groupSucceeded = false;
        break;
    }

    local_path_repair_detail::updateReservationState(
        candidateReservations, replacement, agents[index].id);
    candidatePaths[index] = std::move(replacement);
}
```

For IDs `1` and `2`, the reservation lifecycle is:

```text
committed state
  -> copy once
  -> remove old path 1
  -> remove old path 2
  -> SIPP for 1 against all outsiders
  -> insert new path 1
  -> SIPP for 2 against all outsiders and new path 1
  -> insert new path 2
  -> validate and commit the group
```

Even if the first SIPP result equals that agent's old path, it must be inserted before the next search. Every participant gets a full search; do not stop after changing only the first agent.

The reservation invariant before participant `j` searches is exactly: unchanged paths of nonparticipants plus new paths of earlier participants. Later participants have no reservations yet. Never restore any of their old paths as obstacles during this attempt.

The required helpers remove/add explicit vertex ownership, permanent goal occupancy, and reverse-edge arrival blocks. Reuse their ownership-aware behavior so another agent's overlapping reservation survives removal. Never clear whole cell/edge entries manually or expand an infinite parked tail into explicit timesteps.

Use only the supplied-table SIPP overload with `Permanent`. The path-vector overload would rebuild safe intervals; the transient overload could accept a goal arrival followed by a future collision. The table is unchanged during each SIPP call and updated only between calls.

### 4. Validate, publish, and detect the next conflict

If any SIPP search fails, discard both temporary objects and return the last committed result with unsuccessful metrics. Earlier successfully committed groups remain in that result. Do not restore paths individually through a full rebuild, publish a partially repaired group, or replace failed paths with empty slots.

After all replacements are inserted:

1. Compute `candidateConflicts = getCollision(candidatePaths)` once for the completed group.
2. Check that no global vertex/edge event involves any repaired participant. Other agents may still conflict with each other; those conflicts belong to later iterations.
3. Check progress: the number of global vertex plus edge events must strictly decrease. Treat a failed structural/collision/progress check as a rejected group and return unsuccessful metrics with the committed snapshot.
4. Finish all potentially throwing validation before publication. Move candidate paths, conflicts, and reservations into the result together; follow the existing common engine's no-throw move-assignment checks.
5. Select the next event from the newly computed conflicts. Never reuse an event reference, iterator, or queued selection from the old snapshot.

```cpp
if (groupSucceeded) {
    auto candidateConflicts = getCollision(candidatePaths);
    // Validate participant freedom and strict event-count reduction here.
    // Publish only after all checks and potentially throwing work succeed.
    result.paths = std::move(candidatePaths);
    result.remainingConflicts = std::move(candidateConflicts);
    result.reservations = std::move(candidateReservations);
}
```

The loop terminates successfully when the current conflict set is empty and every required path exists. Under these invariants, a committed group creates no conflicts involving its participants, while conflicts solely among outsiders cannot be created because their paths did not change. The selected event disappears, so each commit strictly reduces the finite event count. A failed attempt stops immediately. No arbitrary iteration cap, retry loop, or local-engine fingerprint machinery is needed for this version.

Preserve vertex and swap exclusion, including same-direction co-occupancy detected as vertex events. Following moves and collision-free cycles remain allowed. A goal must remain safe indefinitely after final arrival.

Propagate input/ownership/allocation exceptions through the existing error boundary; do not disguise programming errors as normal SIPP failure. The committed state remains untouched when an incremental helper throws because it only mutates a disposable copy.

### 5. Share metrics and integrate experiments

Move `updateResultMetrics` from the anonymous namespace into `local_path_repair_detail` and declare it in the common internal header. Preserve its implementation and existing callers' semantics: action cost is `path.size() - 1`, absent paths contribute zero to solver metrics, injustice is the population standard deviation of final-minus-initial costs over available pairs, and duration includes initialization plus repair. Preserve the artifact writer's separate missing-path cost of `-1`.

Register the new class in `src/main.cpp` and show it in CLI usage. Reject explicitly supplied `-threads`, `-continue_if_failed`, and `-localRepairStrategy` for this solver: this proposed first version is sequential, stops on a failed group, and always uses fixed ID priority. Keep existing solvers' option contracts unchanged.

Adapt `supportedSolver`, `validSolverMetadata`, and the failed-run artifact condition in `experiment_utils.cpp`. For the new solver, publish:

```text
solver = FullPathRepairIterativeSolver
continueIfFailed = false
multithreading = false
numThreads = 1
localRepair = false
localRepairStrategy = std::nullopt
```

Its existing `local_repair_strategy` CSV column contains `-`. Extend conflict-file creation to unsuccessful runs where `run.localRepair` is true **or** the solver is `FullPathRepairIterativeSolver`; this covers full-path failures without labeling the algorithm as local repair. Missing-initial-path failures still produce a header-only conflict file when no geometric conflicts remain. Keep the current CSV schema, path alignment, and real-ID normalization.

Preserve exit codes: `0` for successful solving and artifact writing, `1` for an unsuccessful result or artifact-writing failure, and `2` for invalid options/input or solver exceptions. Document this invocation:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver FullPathRepairIterativeSolver \
  -agents 3
```

## Files to create or change during implementation

| File | Planned work |
| --- | --- |
| `include/mapf/solvers/full_path_repair_iterative_solver.hpp` (new) | Public class declaration and result contract. |
| `src/mapf/solvers/full_path_repair_iterative_solver.cpp` (new) | Initial A*, deterministic global-event selection, complete-group removal, sequential full SIPP, atomic publication, and failure handling. Keep small selection/validation helpers private here. |
| `src/mapf/solvers/local_path_repair_solver_common.hpp/.cpp` | Expose the existing metric finalizer through the internal namespace; reuse the reservation helpers unchanged. |
| `src/main.cpp` | Usage, option validation, dispatch, and experiment result mapping. |
| `src/mapf/experiments/experiment_utils.cpp` | Solver allowlist, metadata checks, and full-path failure conflict artifacts. |
| `CMakeLists.txt` | Add the implementation to `mapf`; add/register the new test target with the same include/link conventions as existing solver tests. |
| `tests/solvers/full_path_repair_iterative_solver_test.cpp` (new) | Solver behavior, identity ordering, group semantics, rollback, progress, and result/reservation consistency. |
| `tests/solvers/path_reservation_state_test.cpp` | Extend existing multi-removal coverage with complete-group exclusion and sequential insertion equivalence cases. |
| `tests/experiments/cli_test.cpp`, `tests/experiments/experiment_utils_test.cpp` | New solver dispatch, rejected flags, metadata, successful/failed artifact bundles, and existing-solver regressions. |
| `docs/full_path_repair_iterative_solver.md` (new) | Algorithm, ID/index distinction, reservation lifecycle, failure semantics, metrics, and examples. |
| `docs/experiment_cli_and_results.md`, `readme.md` | List the new solver, fixed priority, accepted options, and full-path failure artifacts. |

No changes are needed to A*/SIPP implementations, the collision detector, reservation data layout, existing solver public APIs, CMake presets, or run/profiling scripts. A future specification can use this plan as its basis; creating it is a separate task.

## Validation and acceptance criteria

Use existing CTest/test-support conventions. Add behavioral cases that distinguish this algorithm from single-agent local repair:

| Scenario | Expected evidence |
| --- | --- |
| Zero agents, one agent, start equals goal, and initially conflict-free paths | Correct aligned results, zero-cost cases, no unnecessary SIPP search, and initial A* paths preserved. |
| An unreachable initial path followed by a reachable agent | Every A* search attempted; missing slot retained; unsuccessful result without repair. |
| Two-agent crossing with instance IDs `[42, 7]`, plus a negative-ID variant | The smaller real ID plans first even when stored later. With no outsiders, its path retains the unconstrained shortest cost because both old paths were removed. The later agent avoids its newly inserted path. |
| Three or more agents sharing a vertex; multiple agents in edge-event direction sets | Every participant is deduplicated and replanned once, including both edge directions. |
| A visitor conflicts with an agent parked at its goal after arrival | The parked owner is included through the global event even when its `byAgent` records omit that collision. Both complete paths are handled. |
| A new goal arrival would be safe only temporarily | Permanent-goal SIPP chooses a later safe arrival or fails; the result never parks through a future visit. |
| Multiple independent or overlapping conflict events, with unaffected agents present | Outsider reservations constrain every search; their paths remain unchanged until selected. Fresh detection drives later groups and event count decreases on every commit. |
| Impossible swap in a two-cell corridor | The first participant can be replanned but the second fails. No partial group is committed; all paths, conflicts, and reservation views equal the prior snapshot. |
| A successful group followed by a failing group | Preserve the first committed repair and roll back only the failed attempt. Initial paths/conflicts remain the original A* snapshot. |
| Shared vertex/edge occupancy, waits, revisits, and changed goal times | Removal preserves outsider contributions, obsolete reservations disappear, and insertion updates the complete new path. |
| Following and collision-free cycle movements | Accepted under project rules; no extra conflict restrictions introduced. |
| Repeated runs and invalid instances | Stable decisions independent of unordered iteration; duplicate starts/goals rejected before search. |
| CLI and artifacts | Correct solver name, costs, `local_repair_strategy=-`, flags, exit codes, failed-run conflict CSVs, and existing solver behavior. |

In reservation tests, compare all four views (`vertex_agents`, `goal_reservations`, safe intervals, blocked edge arrivals) with `buildReservationState` as a **test oracle**. To represent several excluded participants, keep the path vector aligned and empty their slots in the reference input. Compare after all removals, after each insertion, and after a final commit. Retain independent expected interval/edge cases so shared implementation logic cannot conceal errors.

Check that production code calls `buildReservationState` exactly once per solve and never calls `getSafeIntervalsByCell` or a path-vector SIPP overload during repair. Verify this through call-site review and, if runtime evidence is needed, temporary test instrumentation rather than a new public diagnostics API. Per-group work must use one removal per old participant path and one insertion per successful replacement, not full reconstruction. Whole-state copies remain allowed and explicitly measured separately.

Run these commands during implementation:

```bash
cmake --preset normal
cmake --build --preset normal --parallel
ctest --preset normal --output-on-failure
```

Also run the documented CLI example and a failing fixture to inspect artifacts. Acceptance requires the intended ordering and incremental lifecycle, collision-free complete paths on success, a consistent last committed snapshot on failure, and passing existing regressions. No numerical speedup, MAPF completeness, or global makespan/sum-of-costs optimality is promised by fixed-priority repair.

## Deferred improvements

- TODO: Profile reservation/path copies, global conflict detection, and SIPP separately before replacing disposable copies with overlays or adding incremental collision detection.
- TODO: Consider a neutral shared result/reservation module if more full-path solvers are added; reuse the existing types and internal namespace for this version.
- TODO: Consider configurable continuation or alternative priorities only in a separate feature with explicit group rollback and scheduling contracts.
- TODO: While updating CLI documentation, reconcile the existing `RESOLVE_BY_TIME` description with the implementation: the current common engine implements it although the README still describes it as unimplemented.

## prompt

### User request

```text
leia o arquivo task.md e faça o que se pede
```

### Task definition from `task.md`

```text
leia o arquivo .github/ai-instructions.md e faça um plano para a seguinte funcionalidade:

descrição: criação de um solver de nome "FullPathRepairIterativeSolver". Este solver tem uma ideia parecida com o LocalPathRepairIterativeSolver. Primeiro ele calcula os caminhos de todos os agentes de forma iterativa, independente se tiverem conflitos. Em seguida, ele resolve iterativamente todos os conflitos. A forma como ele resolve os conflitos é recalcular o caminho dos agentes envolvidos no conflito, usando o SIPP para achar novos caminhos sem conflitos. O id dos agentes escolhe a ordem de prioridade para resolução dos conflitos, por exemplo, se os agentes 1 e 2 estão envolvidos em um conflito, primeiro é refeito o caminho do agente 1 e depois do agente 2.

Para implementar esse solver, faça o seguinte:
* para calcular os caminhos iniciais dos agentes, use o o AStarSolver (src/mapf/pathfinding/a_star.cpp)
* para achar os conflitos existentes, use a função getCollision de src/mapf/utils.cpp
* depois de achar os caminhos iniciais, use a função buildReservationState (src/mapf/solvers/local_path_repair_solver_common.cpp) para primeiro montar a tabela de intervalos seguros (safe intervals)
* para cada agentes envolvidos no conflitos, use as funções repairSafeIntervalTable para excluir a influencia dos caminhos do agente na tabela de intervalos seguro
* depois que fizer o SIPP para recalcular o caminho de um agente, use a função updateReservationState para atualizar a tabela de intervalos seguros
* não refaça a tabela de intervalos seguros toda vez que achar o caminho de um agente ou quando precisar retirar a influencia de caminhos de um agente
* quando for resolver um conflito, primeiro retire a influência de todos os agentes envolvidos no conflito da tabela de intervalos seguro (safe intervals). Depois, use o SIPP para achar o caminho de um agente, depois atualize a tabela de intervalos seguros com a influência do novo caminho do agente. Repita esse processo para todos os agente do conflito

```
