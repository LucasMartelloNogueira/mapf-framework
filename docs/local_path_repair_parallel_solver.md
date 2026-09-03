# Parallel Initial Planning With Local Path Repair

`mapf::LocalPathRepairParallelSolver` builds unconstrained paths concurrently and then repairs their conflicts sequentially. The solver is intended for correctness-first experiments in which independent shortest paths are inexpensive to obtain and conflicts can often be removed without discarding an entire path.

The design follows the scenario definitions in `docs/local_path_repair.md` and the delayed-suffix transition procedure in `docs/path_sufix_repair_algorithm.md`.

## API

Include:

```cpp
#include "mapf/solvers/local_path_repair_parallel_solver.hpp"
```

Construct the solver with an instance whose lifetime exceeds the solver's lifetime and a positive worker count:

```cpp
mapf::LocalPathRepairParallelSolver solver(instance, 4);
mapf::LocalPathRepairResult result = solver.solve();
```

A zero worker count throws `std::invalid_argument`. An instance with no agents returns a successful empty result.

`LocalPathRepairResult` contains:

- aggregate `Result` metrics;
- paths and path costs in `Instance::getAgents()` order;
- conflicts detected immediately after unconstrained A* planning;
- conflicts remaining at return time; and
- the safe intervals, blocked reverse-edge arrivals, vertex-to-agent membership, and permanent goal arrivals derived from the returned paths.

On success, `remainingConflicts` is empty and `validateSolution(paths)` is true. On an algorithmic failure, the result retains the last completely committed paths and rebuilt reservation state for diagnosis.

## Time And Path Conventions

The first element of a complete path is occupied at absolute time zero. Each following list element represents one move or wait, so path cost is `path.size() - 1`.

The time-offset SIPP overload receives an absolute `startTime`. Its returned segment still begins with the start cell. A segment of size `n` therefore arrives at:

```text
startTime + n - 1
```

Repeated cell pointers represent explicit waits. Reverse-edge reservations are indexed by arrival time and prevent opposite-direction swaps.

The project uses stay-at-target semantics. A complete path may finish only in a safe interval that extends through `SAFE_INTERVAL_INFINITY`; its goal is then reserved from the finite arrival timestep onward. A local bridge endpoint is transient and may use a finite safe interval because the repaired suffix leaves it later.

## Parallel And Sequential Phases

The initial phase submits one independent `AStarSolver` task per agent to a bounded C++20 thread pool. Futures are collected in instance order, so task completion order cannot change path ownership. Workers share only read-only grid data, and every task owns its search state.

The pool is joined before reservation construction. All local repairs then run sequentially. While one agent is active, every other current path is frozen and included in the SIPP reservation table.

## Reservation State

`PathReservationState` groups four synchronized reservation views:

- `safeIntervalTable.safeIntervalsByCell` stores the complement of all vertex reservations;
- `safeIntervalTable.blockedEdgeArrivals` stores reverse directed edges and their blocked arrival times;
- `vertex_agents` maps every finite path cell to the IDs of agents whose paths visit it; and
- `goal_reservations` maps each permanent goal to its arrival timestep.

Before repairing agent `i`, the solver rebuilds a temporary state from every path except `i`. This prevents the obsolete path from blocking its own replacement. After accepting a candidate, the complete state is rebuilt from all paths. Incremental deletion is deliberately avoided because overlapping reservations do not retain per-agent provenance.

## Conflict Selection And Local Window

Agents are processed in instance order. For each agent, the solver selects its earliest explicit conflict. A virtual stay-at-goal occupant is not selected when another agent enters its goal; the moving path is repaired.

For a vertex conflict at time `tc`, the retained prefix ends at `tc - 1` and the old suffix starts at `tc + 1`. The conflicting cell occurrence is removed. For an edge swap arriving at `tc`, the prefix ends at `tc - 1` and the suffix starts at `tc`; the conflicting transition is replaced.

SIPP first tries to bridge the last retained prefix cell to the first suffix cell. If the window fails, the prefix anchor follows this sequence:

```text
a, floor(a / 2), floor(floor(a / 2) / 2), ..., 0
```

No anchor is attempted twice. A candidate is committed only when it is structurally valid, reaches a permanently safe real goal, and moves the active path's earliest conflict to a later time or removes it. Later conflicts may remain and are handled by the next repair iteration.

## Suffix Scenarios

Let `originalSuffixArrivalTime` be the old index of the first suffix cell and `newSuffixArrivalTime` be the bridge's absolute arrival time there.

| Scenario | Condition | Behavior |
| --- | --- | --- |
| 1 | Times are equal | Reuse the old suffix at its original timing. |
| 2.1 | New arrival is earlier and another agent may use a suffix cell | Wait at the first suffix cell until its original time, then reuse the original timing. |
| 2.2 | New arrival is earlier and no other agent uses a suffix cell | Reuse the suffix immediately with the earlier timing. |
| 2.3 | New arrival is later and another agent may use a suffix cell | Validate or repair each suffix transition according to safe intervals and blocked edges. |
| 2.4 | New arrival is later and no other agent uses a suffix cell | Reuse the suffix with the later timing. |

Scenario 2.1 is accepted only if the complete mandated wait remains in one safe interval.

For Scenario 2.3, the current and next suffix cells each either have or do not have potential conflict. The solver handles the four combinations as follows:

- neither has potential conflict: append the original transition after temporal validation;
- only the current cell has potential conflict: its arrival is already valid, so validate and append the transition;
- only the next cell has potential conflict: wait at the current cell until the earliest safe, edge-valid arrival; and
- both have potential conflict: run a time-offset SIPP mini-search between them.

Every move checks the reverse-edge arrival table. Waits must remain inside the current safe interval, and an arrival at the real goal must use an infinite safe interval.

## Full Replanning And Failure

If every local anchor fails, SIPP replans from the true source at time zero to the true destination while all other paths remain frozen. This search requires permanent-goal safety and must be conflict-free before commit.

Failure is returned when initial A* cannot reach a goal, fixed starts coincide, permanent goals are duplicated, every local window fails, or complete fallback fails. Failed candidates never partially update committed paths or reservation structures.

## Metrics

`sumOfCosts` and `makespan` are computed from the returned path costs. `injustice` is the population standard deviation of each agent's final cost increase relative to its initial unconstrained A* cost. `durationSeconds` covers the complete `solve()` call on both success and failure.

## Complexity

For `A` agents, total finite path length `L`, `V` grid cells, and `R` accepted repair commits:

- initial planning runs `A` A* searches with at most the configured worker count active;
- pairwise time-major collision detection is `O(A^2 * makespan)`;
- a complete reservation rebuild is `O(V + L log L)` with per-cell interval sorting; and
- adaptive local repair performs at most logarithmically many anchor attempts before one complete fallback for a selected conflict.
