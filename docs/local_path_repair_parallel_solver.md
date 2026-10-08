# Parallel Initial Planning With Local Path Repair

`mapf::LocalPathRepairParallelSolver` builds unconstrained paths concurrently and then repairs their conflicts sequentially. The solver is intended for correctness-first experiments in which independent shortest paths are inexpensive to obtain and conflicts can often be removed without discarding an entire path.

The local engine splices a SIPP bridge into the active path and updates conflicts over the remaining suffix. The older suffix scenarios in `docs/local_path_repair.md` and `docs/path_sufix_repair_algorithm.md` are no longer used by this implementation.

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
- immutable unconstrained A* paths in `initialPaths`;
- paths and path costs in `Instance::getAgents()` order;
- conflicts detected immediately after unconstrained A* planning;
- conflicts remaining at return time; and
- the safe intervals, blocked reverse-edge arrivals, vertex-to-agent membership, and permanent goal arrivals derived from the returned paths.

On success, `remainingConflicts` is empty and `validateSolution(paths)` is true. On an algorithmic failure, the result retains the last completely committed paths, conflicts, and reservation state for diagnosis.

The shared result and reservation declarations live in
`mapf/solvers/local_path_repair_solver.hpp`. Including the parallel solver
header remains sufficient because it includes this neutral header.

## Time And Path Conventions

The first element of a complete path is occupied at absolute time zero. Each following list element represents one move or wait, so path cost is `path.size() - 1`.

The time-offset SIPP overload receives an absolute `startTime`. Its returned segment still begins with the start cell. A segment of size `n` therefore arrives at:

```text
startTime + n - 1
```

Repeated cell pointers represent explicit waits. Reverse-edge reservations are indexed by arrival time and prevent opposite-direction swaps.

The project uses stay-at-target semantics: every destination is reserved from arrival onward. Intermediate bridge endpoints use `GoalOccupation::Transient`. A bridge that ends the complete path, and the complete fallback, use `GoalOccupation::Permanent`. A copied suffix can still have conflicts, including future visits to its destination; these become repair records for later iterations.

`AStarSippSolver` accepts an explicit policy when reusing a table:

```cpp
auto path = sipp.solve(grid, start, goal, reservations.safeIntervalTable, 0,
    mapf::AStarSippSolver::GoalOccupation::Permanent);
```

The existing five-argument table overload remains transient; the overload accepting other agents' paths remains permanent and constructs its own table. The local engine's full fallback uses the explicit permanent policy with its existing excluded-agent table. For example, a goal safe in `[0,4]` and `[6,+inf]` cannot accept a permanent arrival at 3, even though an intermediate bridge endpoint can. Invalid policy values throw `std::invalid_argument`.

## Parallel And Sequential Phases

The initial phase submits one independent `AStarSolver` task per agent to a bounded C++20 thread pool. Futures are collected in instance order, so task completion order cannot change path ownership. Workers share only read-only grid data, and every task owns its search state.

The pool is joined before reservation construction. All local repairs then run sequentially. While one agent is active, every other current path is frozen and included in the SIPP reservation table.

The sequential repair logic is implemented once in the internal local-repair
common module. `LocalPathRepairParallelSolver` supplies initial paths from its
bounded pool; `LocalPathRepairIterativeSolver` supplies the same paths from
one-at-a-time A* calls on the caller thread. Both then call the common engine,
so repair order, reservations, fallback behavior, metrics, and failure
semantics remain identical.

The iterative counterpart is constructed as follows:

```cpp
#include "mapf/solvers/local_path_repair_iterative_solver.hpp"

mapf::LocalPathRepairIterativeSolver solver(instance);
mapf::LocalPathRepairResult result = solver.solve();
```

## Reservation State

`PathReservationState` groups four synchronized reservation views:

- `safeIntervalTable.safeIntervalsByCell` stores the complement of all vertex reservations;
- `safeIntervalTable.blockedEdgeArrivals` stores reverse directed edges and their blocked arrival times;
- `vertex_agents[cell][time]` contains the `Agent::id` values explicitly visiting that cell at that time, including the last path element; and
- `goal_reservations[cell]` contains `GoalReservation{agentId, arrivalTime}`, covering the permanent tail.

`Instance` guarantees `agents[i].id == i`, assigning IDs in input order and replacing supplied IDs in manually constructed instances. Scenario buckets and positions are preserved. Reservations and conflict events therefore use the same IDs/path indexes. Conflict updates read the prepared reservations by const reference, without ID conversion or an additional state copy. Empty occupancy sets/maps and empty edge-time sets are removed. Every grid cell keeps a safe-interval entry: `[0, SAFE_INTERVAL_INFINITY]` means fully free, and an empty vector means fully blocked. Intervals include both endpoints.

The initial state is built once. Each actual repair attempt makes one complete copy and calls the internal `repairSafeIntervalTable(copy, oldFullPath, agentId)` to exclude that agent. All anchors and the full fallback reuse the same table without changing it during search. After a structurally valid candidate is accepted, `updateReservationState(copy, newFullPath, agentId)` inserts its reservations. Paths, conflicts and reservations are then published together. Rejected attempts discard the copy.

Each helper regenerates safe intervals once per distinct cell in its supplied path, using the union of remaining explicit visits and permanent occupancy. Removing one of two agents at the same time does not free that time. Removing a parked owner also preserves later visits by other agents. Insertion can split intervals, and removal can create, extend or merge them. The complete old and new paths are used so waits shifting a suffix also shift its edge times and goal arrival.

For a move `U -> V` arriving at `t`, the blocked SIPP edge is `{V,U}` at `t`. After exclusion, this entry remains while the owner sets at `(U,t-1)` and `(V,t)` intersect. This preserves shared movements without a separate edge-owner index; waits do not contribute edges.

These internal helpers require a matching complete registered path for removal and prior removal before replacement. Null cells/unsupported lengths throw `std::invalid_argument`; detected ownership mismatches or occupied destination insertion throw `std::logic_error`. Empty paths are no-ops. All contract checks precede mutations; an allocation failure during mutation can leave the disposable copy partial, so it must be discarded. The committed result is never the mutation target.

## Conflict Selection And Local Bridges

`RESOLVE_BY_AGENT` (the default) selects the first conflict in instance order.
`RESOLVE_BY_TIME` compares the first records across agents with deterministic
ordering. A parked owner has no explicit repair record after its final arrival;
the visitor owns the repair record when it enters that destination.

For a vertex conflict at time `t`, the first anchor is `t - 1` and reconnection
is at old index `t + 1`. For an edge swap arriving at `t`, reconnection is at
old index `t`. The next conflict does not limit the repair, so consecutive
conflicts do not invalidate the reconnection automatically. A missing local
reconnection, including a vertex conflict at the last index, triggers the
complete fallback.

Anchors are tried in this order:

```text
a, floor(a / 2), floor(floor(a / 2) / 2), ..., 0
```

The resulting path is the old prefix through the anchor, the bridge without
its first cell, and the old suffix after the reconnection cell. Neither bridge
endpoint is duplicated. The suffix is reused immediately at its shifted time;
there is no separate suffix search or collision rejection before publishing it.
Structural checks preserve the endpoints, legal moves/waits and supported time
range. Previously committed complete path configurations are rejected to avoid
repeating the same state; unusable local candidates lead to the next anchor or
the complete fallback.

`UpdateSolutionConflictsV2` runs after insertion of the replacement reservations.
It removes obsolete events and discovers conflicts in the changed path, including
permanent occupancy at other destinations. It also refreshes old global events
involving the active agent and other paths' visits to its destination, so changing
its final arrival can create or remove conflicts beyond its explicit path end.
Only explicit occupants receive `byAgent` records; parked owners still participate
in global vertex events. Edge swaps use explicit movements only.

## Full Replanning And Failure

If every local anchor fails, SIPP replans from the true source at time zero to the true destination while all other paths remain frozen. This search uses `GoalOccupation::Permanent`; its structurally valid replacement is published through the same incremental update, starting at index zero.

Failure is returned when initial A* cannot reach a goal or both the local attempts and complete fallback fail. Shared starts and duplicate permanent goals are rejected during instance construction. Failed candidates never partially update committed paths or reservation structures.

There is no continuation option. A failed local attempt followed by a failed
complete fallback returns the last committed state immediately.

Both solvers accept the repair strategy directly:

```cpp
mapf::LocalPathRepairParallelSolver parallel(instance, 4, mapf::LocalRepairStrategy::RESOLVE_BY_TIME);
mapf::LocalPathRepairIterativeSolver iterative(instance, mapf::LocalRepairStrategy::RESOLVE_BY_TIME);
```

## Metrics

`sumOfCosts` and `makespan` are computed from the returned path costs. `injustice` is the population standard deviation of each agent's final cost increase relative to its initial unconstrained A* cost. `durationSeconds` covers the complete `solve()` call on both success and failure.

## Complexity

For `A` agents, total finite path length `L`, `V` grid cells, and `T` actual repair attempts:

- initial planning runs `A` A* searches with at most the configured worker count active;
- conflict detection indexes explicit occupancy and movements, with additional sorting of per-agent conflict records;
- the one initial reservation build is bounded by `O(V + L log L)`, with sorting performed per cell;
- each of the `T` attempts copies the reservation state, costing `O(V + L)` in stored entries under expected hash-table costs;
- removal/insertion traverse the supplied paths, scan/sort occupancy only at affected cells, and check remaining owners of affected movements; and
- adaptive local repair performs at most logarithmically many anchor attempts before one complete fallback for a selected conflict.

The temporal owner index increases storage and copy costs. Local regeneration is not simply `O(path length)`: crowded cells and edge endpoint sets require examining other agents too. Candidate path copying, incremental conflict updates and configuration fingerprinting remain separate costs. See the [consolidated reservation documentation](incremental_path_reservatons.md) for the problem, design decisions, validation, and performance limits.

TODO: Optimize the full state copy and affected-event scans only after profiling representative workloads.
