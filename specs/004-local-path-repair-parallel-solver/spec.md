* Created at: 2026-09-03 07:34:08 -03
* Author: lucas
* Last updated at: 2026-09-03 08:01:03 -03
* AI model used: GPT-5 (Codex)

# Spec: Parallel Initial Planning With Local Path Repair

## Source And Precedence

This specification implements the approved plan at
`specs/004-local-path-repair-parallel-solver/plan.md`.

The plan's `adjustment 1` is incorporated into this specification. If the plan is adjusted later, its newest adjustment overrides conflicting requirements in this specification until the specification is updated.

## Objective

Add a MAPF solver that:

1. computes one unconstrained shortest path per agent in parallel with A*;
2. detects all conflicts in those paths;
3. repairs conflicting paths sequentially with local, earliest-arrival SIPP searches;
4. expands a failed local repair window toward the source;
5. falls back to a complete origin-to-goal SIPP replan when local repair fails; and
6. returns paths, costs, conflicts, metrics, safe intervals, edge reservations, vertex membership, and permanent goal reservations that all describe the same committed solution state.

Only the initial independent A* searches may run concurrently. Conflict repair and every mutation of solution-derived state must be sequential.

## Scope

This feature includes:

- a time-offset SIPP overload that receives an existing `SafeIntervalTable`;
- a bounded C++20 thread pool for the initial A* phase;
- path-order-independent vertex, edge-swap, and stay-at-target collision reporting;
- deterministic rebuilding of all reservation state;
- local path splitting and adaptive-window repair;
- all suffix timing scenarios defined in `docs/local_path_repair.md`;
- the transition-by-transition suffix procedure in `docs/path_sufix_repair_algorithm.md`;
- complete-path SIPP fallback;
- result metrics and diagnostic conflict sets;
- focused automated tests and solver documentation.

The implementation must use the current path representation:

```cpp
std::vector<std::list<mapf::Cell*>>
```

No third-party runtime library is required. Use C++20 standard-library concurrency primitives and CMake's `Threads::Threads` target.

## Terminology And Time Model

- A path list index is its absolute timestep when the path starts at time zero.
- A move or wait consumes one timestep.
- Path cost is `path.size() - 1`; an empty path has no valid MAPF cost.
- Under stay-at-target semantics, an agent occupies its final path cell indefinitely after its finite path ends.
- A **prefix** is the retained path segment before a conflict. Its retained cells are already valid.
- A **suffix** is the old path segment after the removed conflicting location or transition. It may require timing repair.
- A **bridge** is the SIPP path segment joining a retained prefix anchor to the first retained suffix cell.
- `B` is the first retained suffix cell.
- `G` is the active agent's real destination.
- `originalSuffixArrivalTime` is `B`'s index in the old path.
- `newSuffixArrivalTime` is the bridge's absolute arrival time at `B`.
- A **frozen path** is any current path other than the active path being repaired.
- A **transient goal** is a bridge or mini-path endpoint that the active agent will leave later.
- A **permanent goal** is the agent's real destination, which must be safe forever from its arrival time.

For a time-offset SIPP segment starting at absolute time `t`, the returned list contains the start cell as its first item. Its absolute arrival time is therefore:

```cpp
t + static_cast<int>(segment.size()) - 1
```

## Public API Requirements

### Safe-interval infinity

Expose one shared infinity sentinel in `include/mapf/pathfinding/sipp/safe_interval_table.hpp`:

```cpp
inline constexpr int SAFE_INTERVAL_INFINITY =
    std::numeric_limits<int>::max() / 4;
```

Include `<limits>` in that header. SIPP and reservation builders must use this constant instead of defining source-local alternatives.

### Time-offset SIPP

Keep the existing `AStarSippSolver` API and add this overload in `include/mapf/pathfinding/a_star_sipp.hpp`:

```cpp
std::list<Cell*> solve(
    Grid& grid,
    Cell* start,
    Cell* goal,
    const SafeIntervalTable& safeIntervalTable,
    int startTime
);
```

The table must be passed by const reference. The overload treats `goal` as a transient endpoint and must return the earliest-arriving valid segment.

The existing overload that receives other-agent paths remains source compatible and continues to represent a complete origin-to-destination plan. It must build a reservation table and use the shared SIPP implementation with a permanent-goal acceptance policy. A permanent goal is acceptable only when the selected goal safe interval ends at `SAFE_INTERVAL_INFINITY`.

Use one private search implementation with an internal transient/permanent goal policy. Do not duplicate the SIPP algorithm between overloads.

### Reservation state

Declare the solution-derived reservation types in the new solver header:

```cpp
using VertexAgents = std::unordered_map<Cell*, std::unordered_set<int>>;
using GoalReservations = std::unordered_map<Cell*, int>;

struct PathReservationState {
    SafeIntervalTable safeIntervalTable;
    VertexAgents vertex_agents;
    GoalReservations goal_reservations;
};
```

`vertex_agents[cell]` contains real `Agent::id` values, not vector indexes. `goal_reservations[cell]` contains the absolute timestep at which the owning agent reaches that cell and remains there.

### Solver result

Add this result type in `include/mapf/solvers/local_path_repair_parallel_solver.hpp`:

```cpp
struct LocalPathRepairResult {
    Result metrics;
    std::vector<std::list<Cell*>> paths;
    std::vector<int> pathCosts;
    SolutionConflicts initialConflicts;
    SolutionConflicts remainingConflicts;
    PathReservationState reservations;
};
```

Required semantics:

- `paths[i]` belongs to `instance.getAgents()[i]`.
- `pathCosts[i]` belongs to the same agent and equals `paths[i].size() - 1` for a non-empty path.
- `initialConflicts` is captured immediately after all initial A* futures have been collected.
- `remainingConflicts` describes the paths at return time.
- `metrics.success` is true only when every agent has a non-empty path and `remainingConflicts` is empty.
- `reservations` must describe the returned `paths`, including when solving returns an algorithmic failure after one or more successful repairs.

### Solver class

Add this class in the same header:

```cpp
class LocalPathRepairParallelSolver {
public:
    LocalPathRepairParallelSolver(
        const Instance& instance,
        std::size_t numberOfThreads
    );

    LocalPathRepairResult solve();

private:
    const Instance& instance;
    std::size_t numberOfThreads;
};
```

The solver must not own or mutate the `Instance`. The caller must keep the instance alive for the solver's lifetime. Thread-pool and repair helper types must remain private to the implementation file.

Constructing the solver with `numberOfThreads == 0` must throw `std::invalid_argument`. An instance with no agents is valid and must return an empty successful solution with zero metrics and empty reservation/conflict structures.

## Collision Reporting Requirements

Refactor `getCollision` in `src/mapf/utils.cpp` so results do not depend on input path order.

1. Convert each non-empty list to an indexable vector once.
2. Let the makespan be the largest non-empty path cost.
3. Evaluate each timestep from zero through the makespan.
4. For a non-empty path, define `positionAt(path, t)` as:
   - `path[t]` when `t < path.size()`;
   - `path.back()` otherwise.
5. For every unordered path pair at each timestep, emit a vertex conflict when both `positionAt` values are the same cell.
6. For every unordered path pair and timestep greater than zero, emit an edge conflict when the agents traverse the same non-degenerate edge in opposite directions.
7. Emit each pairwise event exactly once. Three agents sharing a cell may therefore produce three pairwise `CellConflict` records even though those records have identical cell/time values.
8. Skip empty paths; they do not occupy a cell.

Keep the existing public function names and signatures:

```cpp
mapf::SolutionConflicts getCollision(
    const std::vector<std::list<mapf::Cell*>>& paths
);

bool validateSolution(
    const std::vector<std::list<mapf::Cell*>>& paths
);
```

`validateSolution` must remain a thin wrapper over `getCollision` and return true only when both conflict vectors are empty.

Collision records do not need agent IDs. The local-repair solver must map conflict cell/edge and time data back to involved path indexes with private helpers that use the same `positionAt` semantics.

## Time-offset SIPP Behavior

The shared SIPP implementation must satisfy all of the following:

- Return an empty path for null endpoints, blocked endpoints, a negative `startTime`, a missing start-cell interval, or a start cell that is unsafe at exactly `startTime`.
- Initialize the start node's `time` and `g` to `startTime`.
- Use absolute times for `SippNode::time`, `g`, `bestG`, safe-interval lookup, waiting limits, and `blockedEdgeArrivals` checks.
- Continue to use Manhattan distance as the heuristic.
- Minimize the absolute arrival time at the selected goal interval.
- Reconstruct a segment whose first cell is `start`, with explicit repeated cells for every wait.
- Never wait beyond the end of the current cell's safe interval.
- Reject an opposite-direction edge traversal when the candidate arrival time is present in `blockedEdgeArrivals`.
- Return a one-cell segment for `start == goal` only when the start state and the selected goal policy are valid.
- Preserve the behavior and source compatibility of the existing path-based overload.

For a transient goal, any safe interval containing the arrival time is acceptable. For a permanent goal, only a safe interval ending at `SAFE_INTERVAL_INFINITY` is acceptable.

## Reservation Rebuild Requirements

Implement one deterministic builder in `src/mapf/solvers/local_path_repair_parallel_solver.cpp`, equivalent to:

```cpp
PathReservationState buildReservationState(
    Grid& grid,
    const std::vector<Agent>& agents,
    const std::vector<std::list<Cell*>>& paths,
    std::optional<std::size_t> excludedPathIndex = std::nullopt
);
```

For every included, non-empty path, the builder must:

1. insert the corresponding real agent ID into `vertex_agents` for every finite path cell;
2. reserve every finite `(cell, timestep)` occupancy;
3. insert the reverse directed edge and arrival timestep into `blockedEdgeArrivals` for every move;
4. reserve the final cell from `path.size() - 1` through `SAFE_INTERVAL_INFINITY`;
5. store that arrival time in `goal_reservations`; and
6. merge blocked intervals and complement them into sorted, non-overlapping safe intervals for every grid cell.

Repeated visits by one agent must still create only one ID entry in a cell's set. Duplicate final goal cells are invalid under stay-at-target semantics and must cause solving to fail rather than silently overwrite `goal_reservations`.

The solver uses two kinds of snapshots:

- the committed snapshot includes every current path and is returned in the result;
- the repair snapshot excludes the active agent's path and freezes all other current paths.

Never try to remove one agent incrementally from `safeIntervalsByCell` or `blockedEdgeArrivals`. Those structures do not retain enough ownership information. Build a new state from paths, validate it, and commit the complete new snapshot only after the candidate path is accepted.

## Initial Parallel Planning

Implement a bounded thread pool private to `local_path_repair_parallel_solver.cpp` using:

- `std::jthread` workers;
- a `std::queue<std::function<void()>>` task queue;
- `std::mutex` and `std::condition_variable` for synchronization; and
- `std::packaged_task` plus `std::future` for return values and exception propagation.

The initial phase must execute as follows:

1. Start the solver duration timer.
2. Read agents in `Instance::getAgents()` order.
3. Allocate the result path vector at the exact agent count before starting work.
4. Create `min(numberOfThreads, max(agentCount, 1))` workers.
5. Submit exactly one task per agent.
6. In each task, create a separate `AStarSolver` and compute an unconstrained path from that agent's start to goal.
7. Share only read-only grid/cell state across tasks. Search containers and solver objects must not be shared.
8. Store futures in instance-agent order and call `get()` in that same order so completion order cannot change path ownership.
9. Join/destroy all workers before collision detection, reservation rebuilding, or repair begins.
10. Populate `pathCosts`, `initialConflicts`, `remainingConflicts`, and the committed reservation snapshot from the collected paths.
11. If any initial path is empty, return `metrics.success == false` without entering repair. Preserve the collected paths for diagnostics.

The legacy grid/pathfinding API is not const-qualified. The implementation may obtain the current non-const grid reference once, as existing solvers do, but the initial tasks must call only grid operations that do not modify cells or grid storage. No worker may observe or mutate solution/reservation state.

Exceptions raised in a planning task must be delivered by the corresponding future. Algorithmic inability to find a path is represented by an empty path and a failed result.

## Terminal Endpoint Conflicts

Before attempting local repair:

- fail when two non-empty paths occupy the same start cell at timestep zero, because fixed starts cannot be repaired backward in time;
- fail when two agents have the same destination, because both would reserve it forever.

Return the current diagnostic paths, conflict sets, costs, and reservation state without partially applying a repair.

## Conflict Ownership And Repair Order

Repair agents sequentially in `Instance::getAgents()` order. For the active agent, repeatedly recompute the current conflicts and choose its earliest explicit conflict.

An agent is explicitly involved in a vertex conflict at time `tc` only when `tc` is inside its finite path and its cell at that index matches the conflict cell. An agent represented only by virtual stay-at-goal occupancy is not selected for repair; the agent that explicitly enters or occupies that goal later must be selected.

An agent is involved in an edge conflict arriving at `tc` when its finite transition from `tc - 1` to `tc` matches one direction of the reported edge.

When multiple explicit participants are eligible, instance order is the deterministic tie-breaker. Conflicts at the same time must also use a deterministic ordering, with vertex conflicts before edge conflicts and stable cell/edge coordinate ordering.

Paths of already repaired agents and not-yet-repaired agents are both frozen while one candidate for the active agent is computed.

## Path Splitting

Use these split indexes:

| Conflict | Retained prefix ends at | Old suffix starts at |
| --- | ---: | ---: |
| Vertex conflict at `tc` | `tc - 1` | `tc + 1` |
| Edge swap arriving at `tc` | `tc - 1` | `tc` |

For a vertex conflict, the conflicted cell occurrence must be removed from the repaired local segment. For an edge conflict, the conflicting transition must be replaced, while its old arrival cell may remain the first suffix cell.

Prefix and suffix anchors are included in their respective segments. When combining prefix, bridge, and suffix, omit duplicate copies of shared anchor cells so each list element still represents exactly one timestep.

If a required prefix or suffix side does not exist, skip local bridging and proceed directly to full-path replanning.

## Adaptive Repair Window

For an initial retained prefix endpoint index `a` and suffix start index `b`:

1. Build a repair reservation snapshot that excludes the active agent.
2. Attempt time-offset SIPP from `oldPath[a]` at absolute time `a` to `oldPath[b]`.
3. Calculate `newSuffixArrivalTime` as `a + bridge.size() - 1`.
4. Apply the appropriate suffix scenario.
5. Structurally and temporally validate the complete candidate against all frozen paths.
6. Accept the candidate only if the active agent's earliest explicit conflict moves to a later timestep or disappears.
7. If bridging fails, suffix handling fails, or the candidate makes no progress, replace `a` with `floor(a / 2)` and retry.
8. Attempt anchor zero at most once. Track attempted anchors so halving cannot repeat zero or any other anchor.
9. If no local window succeeds, perform a complete origin-to-real-goal SIPP replan at time zero against every frozen path.
10. If full replanning also fails, return a failed result without changing the last committed paths or reservations.

The solver may commit a candidate that still contains a known later conflict. It must not commit a candidate with a new or remaining active-agent conflict at or before the conflict currently being repaired. The repair loop then handles the next later conflict. This is how a path with `k` conflicts becomes valid incrementally.

Within one active-agent repair sequence, keep fingerprints containing the candidate cell sequence and its earliest active conflict. A repeated fingerprint must trigger window expansion or full fallback, never an infinite loop.

## Suffix Scenario Requirements

Determine `hasPotentialSuffixConflict` from the committed `vertex_agents` snapshot. It is true when any cell from `B` through `G` contains an agent ID different from the active agent's ID.

Apply the following behavior:

| Scenario | Condition | Required behavior |
| --- | --- | --- |
| 1 | `newSuffixArrivalTime == originalSuffixArrivalTime` | Append the old suffix at its original timing. The current repair is valid through the next conflict, or through `G` if there is no next conflict. |
| 2.1 | `newSuffixArrivalTime < originalSuffixArrivalTime` and another agent may use the suffix | Wait at `B` for exactly `originalSuffixArrivalTime - newSuffixArrivalTime` timesteps, then append the old suffix at its original absolute timing. |
| 2.2 | `newSuffixArrivalTime < originalSuffixArrivalTime` and no other agent uses the suffix | Append the complete old suffix immediately, shifted earlier. |
| 2.3 | `newSuffixArrivalTime > originalSuffixArrivalTime` and another agent may use the suffix | Repair the suffix transition by transition as specified below. |
| 2.4 | `newSuffixArrivalTime > originalSuffixArrivalTime` and no other agent uses the suffix | Append the complete old suffix shifted later; the delay increases final cost. |

For Scenario 2.1, every wait at `B` must remain inside one safe interval, and blocked-edge checks still apply when leaving `B`. If the mandated wait is unsafe, reject that window and expand or fall back.

For Scenarios 1 and 2.1, it is valid to build the complete path using the old suffix and then immediately detect the next conflict. The candidate may be committed only when the earliest active conflict has advanced beyond the conflict just repaired.

Every scenario result must be checked against frozen paths. `vertex_agents` is a potential-conflict optimization, not a substitute for temporal collision validation.

## Scenario 2.3 Suffix Repair

Start at `B` at `newSuffixArrivalTime` and process each original suffix transition `(current, next)` in order. Determine potential conflict per cell by checking for an agent ID other than the active ID in `vertex_agents`.

| Current cell | Next cell | Operation |
| --- | --- | --- |
| No potential conflict | No potential conflict | Append the original one-timestep transition. |
| Potential conflict | No potential conflict | Treat the current arrival as already validated and append the one-timestep transition. |
| No potential conflict | Potential conflict | Find the earliest safe arrival at `next`; wait explicitly at `current` if necessary. |
| Potential conflict | Potential conflict | Run time-offset SIPP from `current` at its actual absolute time to `next` and append the returned mini-path without duplicating `current`. |

All four cases additionally require:

- every direct transition is either a wait or a four-connected grid edge;
- every move checks `blockedEdgeArrivals`, including transitions whose endpoint membership suggests no potential conflict;
- a wait remains within the current cell's selected safe interval;
- a transition into `next` arrives inside one of `next`'s safe intervals;
- if `next == G`, the selected interval extends through `SAFE_INTERVAL_INFINITY`;
- mini-path detours may leave the old suffix and must be preserved in the candidate and later reservation rebuild; and
- any failure rejects the current local window without changing committed state.

## Candidate Validation And Atomic Commit

Before accepting any candidate, verify that:

- it is non-empty;
- it starts at the active agent's true start and ends at its true goal;
- all cells are non-null and free;
- every consecutive pair is either the same cell or a valid four-connected move;
- its timing does not create a vertex, edge-swap, or stay-at-target conflict at or before the repaired conflict time;
- its earliest explicit conflict is strictly later than the previous earliest conflict or is absent; and
- when no later repair is expected, its real destination remains safe indefinitely.

Create the candidate path, candidate costs, and candidate full reservation state in local objects. Only after all checks pass may the solver replace `paths[i]` and the committed reservation state. A rejected bridge, suffix repair, or full replan must leave every committed structure unchanged.

After each accepted commit:

1. replace only the active agent's path;
2. refresh its path cost;
3. rebuild the complete reservation state from every current path;
4. recompute current conflicts;
5. refresh aggregate metrics; and
6. continue until the active agent has no explicit conflicts.

## Full-path Fallback

Full fallback must:

- start at the active agent's true source at absolute time zero;
- target the active agent's true destination;
- reserve every frozen path, including permanent goal occupancy;
- use the permanent-goal SIPP policy;
- return an explicit complete path; and
- be validated as conflict-free against all frozen paths before commit.

The implementation may invoke the existing path-vector SIPP overload with a vector containing all frozen paths, because that overload builds the required table and applies permanent-goal acceptance.

If full fallback fails, `solve()` returns `metrics.success == false`, retains the last committed path and reservation snapshot, and populates `remainingConflicts` from that state.

## Metrics And Result State

Measure `metrics.durationSeconds` with `std::chrono::steady_clock` from entry to `solve()` through the final success or failure decision.

For every returned state:

- `pathCosts` stays index-aligned with `paths`;
- a non-empty path's cost is `path.size() - 1`;
- an empty diagnostic path uses cost `0`;
- `sumOfCosts` is the sum of `pathCosts`;
- `makespan` is the maximum path cost, or zero when there are no paths;
- `remainingConflicts` is recomputed from the returned paths; and
- `reservations` is rebuilt from all non-empty returned paths.

Calculate injustice as the population standard deviation of:

```text
finalPathCost[i] - initialOptimalPathCost[i]
```

The initial parallel A* paths provide `initialOptimalPathCost`. If an initial A* path is missing, return failure with injustice `0.0`. For an empty agent set, injustice is also `0.0`.

On success, `remainingConflicts` must be empty and `validateSolution(paths)` must return true. On algorithmic failure, preserve diagnostic paths and synchronized derived state instead of returning partially rebuilt structures.

## Files To Create Or Change

### `include/mapf/pathfinding/a_star_sipp.hpp`

- Add the five-argument time-offset `solve` overload.
- Keep existing public declarations source compatible.
- Declare one shared private implementation with an internal goal policy.

### `src/mapf/pathfinding/a_star_sipp.cpp`

- Replace the source-local infinity constant with `SAFE_INTERVAL_INFINITY`.
- Extract shared search behavior.
- Implement absolute-time initialization, validation, expansion, edge checks, and reconstruction.
- Distinguish transient bridge endpoints from permanent real goals.
- Remove the existing TODO about receiving a `SafeIntervalTable`.

### `include/mapf/pathfinding/sipp/safe_interval_table.hpp`

- Include `<limits>`.
- Expose `SAFE_INTERVAL_INFINITY`.
- Keep existing safe interval and blocked edge types compatible.

### `include/mapf/solvers/local_path_repair_parallel_solver.hpp`

- Create `VertexAgents`, `GoalReservations`, `PathReservationState`, `LocalPathRepairResult`, and `LocalPathRepairParallelSolver`.
- Include the types needed by the public API.
- Document index alignment, result ownership, and failure semantics.
- Do not expose thread-pool internals.

### `src/mapf/solvers/local_path_repair_parallel_solver.cpp`

- Implement the bounded thread pool and indexed initial planning.
- Implement reservation rebuilding and active-agent exclusion.
- Implement deterministic conflict ownership and ordering.
- Implement split indexes, splicing, adaptive anchors, all suffix scenarios, Scenario 2.3 transition handling, full fallback, cycle detection, atomic commit, and metrics.

Use the `.cpp` extension. The task's reference to a `.c` file is a typo because this project and solver use C++20.

### `include/mapf/utils.hpp`

- Keep the `getCollision` spelling and signature.
- Do not add the misspelled `getColission` API.
- Add no public participant-mapping helper unless it is independently reusable; private solver helpers are preferred.

### `src/mapf/utils.cpp`

- Replace path-order-dependent collision lookup with the time-major pairwise implementation.
- Preserve `validateSolution` as a wrapper.

### `CMakeLists.txt`

- Add `src/mapf/solvers/local_path_repair_parallel_solver.cpp` to the `mapf` library.
- Call `find_package(Threads REQUIRED)`.
- Link `mapf` with `Threads::Threads`.
- Enable CTest and register the focused test executables.

### `tests/pathfinding/a_star_sipp_test.cpp`

Test:

- nonzero start time;
- a blocked or missing start interval;
- explicit wait reconstruction;
- reverse-edge arrival blocking;
- earliest arrival;
- transient bridge endpoints;
- permanent real goals; and
- compatibility of the existing path-vector overload.

### `tests/utils_collision_test.cpp`

Test:

- order-independent stay-at-goal detection;
- vertex conflicts;
- edge swaps;
- unequal path lengths;
- multiple path pairs at one event;
- three-agent conflicts;
- empty paths; and
- conflict-free paths.

### `tests/solvers/local_path_repair_parallel_solver_test.cpp`

Test:

- deterministic path-to-agent alignment;
- one thread, fewer threads than agents, and more threads than agents;
- Scenarios 1, 2.1, 2.2, 2.3, and 2.4;
- all four Scenario 2.3 transition combinations;
- midpoint-window success after a smaller window fails;
- anchor-zero attempt and complete fallback;
- multiple sequential conflicts for one agent;
- vertex, edge-swap, and stay-at-target repair;
- removal of old vertex and edge reservations after a commit;
- insertion of new reservations and changed goal arrival/cost;
- deterministic repeated runs;
- zero threads;
- unreachable initial path;
- shared starts;
- duplicate goals;
- failed repair;
- an empty instance; and
- start-equals-goal.

Every successful solver fixture must assert both `validateSolution(result.paths)` and empty `result.remainingConflicts`.

### Test scenario comments

Every individual scenario in all three test files must have a concise source-code comment immediately before the test case or scenario block. Each comment must identify what scenario is being tested and state the expected behavior or result. A generic file-level comment does not satisfy this requirement.

### `docs/local_path_repair_parallel_solver.md`

Document:

- public construction and result APIs;
- absolute time and path-index conventions;
- stay-at-target reservation semantics;
- parallel initial planning versus sequential repair;
- adaptive-window anchor sequence;
- the five suffix scenarios;
- complete fallback and failure behavior;
- returned reservation structures; and
- high-level complexity.

Reference `docs/local_path_repair.md` and `docs/path_sufix_repair_algorithm.md`; do not overwrite them.

### `readme.md`

Add a testing section that documents how to configure and compile the project and how to run the complete test suite. It must include at least:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The commands must be shown from the repository root and must match the CMake/CTest configuration delivered by this feature.

## Implementation Sequence

1. Make `getCollision` time-major, pairwise, and order independent; add collision tests.
2. Expose `SAFE_INTERVAL_INFINITY`.
3. Refactor SIPP into shared transient/permanent search behavior.
4. Add the five-argument time-offset overload and its tests.
5. Define the new solver and result public types.
6. Implement and test complete reservation rebuilding, including active-agent exclusion.
7. Implement the private bounded thread pool and deterministic future collection.
8. Add terminal endpoint validation and capture initial conflicts.
9. Implement deterministic conflict ownership, split indexes, path splicing, and adaptive anchors.
10. Implement Scenario 1 and Scenarios 2.1, 2.2, and 2.4.
11. Implement all four transition cases for Scenario 2.3.
12. Add complete permanent-goal fallback and repeated-state protection.
13. Add atomic commit, cost/metric refresh, and final validation.
14. Add a source-code comment identifying every test scenario and its expected behavior or result.
15. Register all tests with CTest and link threads portably.
16. Update `readme.md` with the configure, build, and test commands.
17. Add the solver documentation.
18. Run configure, build, and all tests.

## Acceptance Criteria

- `specs/004-local-path-repair-parallel-solver/spec.md` is the implementation source of truth for the approved plan.
- The new solver accepts a `const Instance&` and a positive thread count.
- Zero threads throws `std::invalid_argument`.
- An empty instance returns a successful empty solution.
- Initial A* searches use a bounded thread pool and remain aligned with instance-agent order.
- Local repair never runs concurrently.
- `getCollision` reports pairwise vertex, opposite-edge-swap, and stay-at-target conflicts independent of path order.
- The new SIPP overload accepts an existing table and absolute start time and returns an earliest-arrival segment with explicit waits.
- Existing SIPP callers remain source compatible.
- Every real destination is accepted only in a safe interval that extends to `SAFE_INTERVAL_INFINITY`.
- Every repair table excludes the active obsolete path and includes all frozen paths.
- Vertex repair removes the conflicting cell occurrence, and edge repair removes the conflicting transition.
- Adaptive anchors move toward the source without repetition and full replanning is attempted after local exhaustion.
- Scenario 1 and Scenarios 2.1 through 2.4 follow the defined timing rules.
- Scenario 2.3 handles all four potential-conflict combinations and always checks edge reservations.
- A path with multiple conflicts is repaired progressively until no conflict remains or fallback fails.
- Accepted repairs update paths, costs, metrics, safe intervals, blocked edge arrivals, vertex membership, and goal reservations as one committed state.
- No reservation from a replaced path remains after rebuilding.
- A failed attempt cannot partially modify committed result state.
- A successful result has empty `remainingConflicts` and passes `validateSolution`.
- Shared starts and duplicate permanent goals fail deterministically.
- Results are deterministic across repeated runs despite parallel initial planning.
- Every test scenario has a source-code comment identifying the scenario and its expected behavior or result.
- `readme.md` documents the repository-root commands for configuring, compiling, and running all tests with failure output.
- `cmake -S . -B build` succeeds.
- `cmake --build build` succeeds.
- `ctest --test-dir build --output-on-failure` succeeds.
- `docs/local_path_repair_parallel_solver.md` documents the delivered behavior.

## Out Of Scope

- Parallel local-repair execution.
- Globally optimal multi-agent replanning.
- Adding agent IDs to `CellConflict`, `EdgeConflict`, or `SolutionConflicts`.
- Introducing a third-party thread-pool or test framework.
- Incremental deletion from reservation structures.
- Changing the `std::list<Cell*>` path representation.
- Repairing fixed shared starts or duplicate permanent goals.

## TODO

TODO: Profile complete reservation rebuilding and pairwise collision detection only after correctness tests are passing; do not add incremental reservation deletion in this feature.

TODO: Extract a reusable project-wide thread pool only if a later feature needs the same abstraction.

TODO: Consider adding participant indexes to conflict records in a later API change; keep mapping private in this feature.

## Adjustments

### adjustment 1

* Adjusted at: 2026-09-03 08:01:03 -03

#### Prompt

```text
faça as seguintes alterações:

* atualize o readme para botar comandos de como compilar e rodar os testes
* para cada cenário de teste, bote comentarios falando qual é o cenario testado
* mude os nomes "newBTime" para "newSuffixArrivalTime  e "originalBTime" para "originalSuffixArrivalTime"

Faça essas alterações nos arquivos specs/004-local-path-repair-parallel-solver/plan.md e specs/004-local-path-repair-parallel-solver/spec.md e grave essas observações na seção "Adjustments" desses arquivos também
```

#### Changes

- README instructions:
  - Before: the specification required solver documentation but did not require build and test commands in `readme.md`.
  - After: `readme.md` must document `cmake -S . -B build`, `cmake --build build`, and `ctest --test-dir build --output-on-failure`.
- Test scenario documentation:
  - Before: the specification listed test coverage without requiring comments for individual scenarios.
  - After: every test scenario must have a concise source-code comment identifying the scenario and its expected behavior or result.
- Suffix timing names:
  - Before: `newBTime` and `originalBTime`.
  - After: `newSuffixArrivalTime` and `originalSuffixArrivalTime`.
- Last-updated metadata:
  - Before: `2026-09-03 07:34:08 -03`.
  - After: `2026-09-03 08:01:03 -03`.
