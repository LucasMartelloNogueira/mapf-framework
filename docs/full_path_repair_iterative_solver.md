# FullPathRepairIterativeSolver

`mapf::FullPathRepairIterativeSolver` computes independent A* paths for every
agent, then replans complete paths for the participants of each conflict using
SIPP. Both stages run sequentially.

```cpp
#include "mapf/solvers/full_path_repair_iterative_solver.hpp"

mapf::FullPathRepairIterativeSolver solver(instance);
mapf::LocalPathRepairResult result = solver.solve();
```

The solver stores a reference to the instance. Paths contain non-owning cell
pointers, so the instance's grid must remain alive while those paths are used.
Calling `solve()` again starts with fresh paths and reservations.

## Initial paths and conflict selection

Initial searches use `AStarSolver` without considering other agents. Every
search is attempted, including agents after an unreachable one. Initial paths
and their conflicts are preserved in `initialPaths` and `initialConflicts`.
If any initial path is missing, the solver returns an unsuccessful result
without attempting repair. A zero-agent instance succeeds.

`getCollision` supplies the global vertex and edge events. A vertex group
contains every participant of that event; a swap group contains the union of
both edge directions. Parked goal owners remain participants after their
explicit paths end, even when their `byAgent` records omit that later event.
Following moves and collision-free cycles are allowed.

Events are ordered by `(minimum participant ID, time, kind, x1, y1, x2, y2)`.
Vertex events precede edge events when the earlier fields tie; edge endpoints
use the detector's canonical coordinate order. Within the selected event,
participants are replanned in ascending `Agent::id` order.

`Instance` assigns IDs in input order, so `agents[i].id == i`, including for
manually constructed instances. Conflict participants, reservation ownership,
and priority use those same IDs/path indexes. Slot `0` has priority over slot
`1`; reordering the input agents changes their priority. Result vectors always
retain instance order. Scenario buckets have no role in priority.

## Reservation lifecycle

The solver calls `buildReservationState` once after the initial searches.
Each conflict attempt opens a `ReservationTransaction` on the current state
and a `PathTransaction` on the current paths. Both utilities are implemented in
[solver_utils.cpp](../src/mapf/solvers/solver_utils.cpp), with declarations in
[solver_utils.hpp](../src/mapf/solvers/solver_utils.hpp):

```text
open reservation and path transactions for the whole group
save affected reservation entries and remove every participant's old complete path
for each participant, in ascending ID order:
    run SIPP from the original start at time zero to the original goal
    save additional affected entries and insert the new complete path
    swap the old path into the undo log and install the replacement
detect conflicts in the temporarily updated paths
validate and commit the whole group, or restore both transactions
```

Before the first search, all group members have been removed. Each search
therefore sees unchanged paths of nonparticipants plus replacements already
inserted for earlier group members. Even an unchanged replacement is inserted
before the next search.

The shared helpers preserve other owners' shared vertex and edge
contributions, update permanent goal reservations, and regenerate intervals
only on affected cells. They receive whole paths with absolute times starting
at zero, including waits and revisits. Only affected reservation entries are
copied; original map nodes are retained for rollback. Path changes transfer
list nodes through swaps, without copying the entire path vector or its lists.

SIPP uses the existing supplied-table overload with
`AStarSippSolver::GoalOccupation::Permanent`. A destination must remain safe
after final arrival. The table stays fixed during a search and is updated
between searches. There is no full reservation rebuild in repair or rollback.

## Publication, failure, and termination

The group is accepted only after all replacements exist, no global conflict
involves a repaired participant, and the total event count strictly decreases.
Conflicts solely among unchanged nonparticipants may remain for later groups.
After acceptance, the new conflict snapshot is moved into the result and both
transactions are committed. These operations cannot throw. Selection then uses
the new conflict snapshot.

If one participant has no valid replacement or group validation fails, both
transactions are explicitly rolled back before computing failure metrics and
returning. This also protects builds without named return value optimization.
The returned result retains all earlier committed groups and every old path
from the failed group. Dependency exceptions propagate; transaction destructors
restore the original paths and reservation entries without allocation.

Every successful group removes an event without creating conflicts involving
its participants, while outsiders' paths remain unchanged. The event count
decreases on each commit, so no retry history or arbitrary iteration cap is
needed. This fixed-priority algorithm may fail on a solvable MAPF instance; it
does not guarantee global makespan or sum-of-costs optimality.

## Results and experiments

The existing `LocalPathRepairResult` holds original paths/conflicts and the
last committed paths/conflicts/reservations. Nonempty path costs count actions
(`size - 1`); empty slots contribute zero to solver metrics. The CSV writer
marks missing per-agent path costs as `-1`. Injustice is the population
standard deviation of final-minus-initial action costs over available pairs.
Solver duration includes initialization and repair, excluding serialization.

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver FullPathRepairIterativeSolver \
  -agents 3
```

The solver accepts the four required CLI flags only. Explicit `-threads` and
`-localRepairStrategy` are rejected, including values that resemble defaults.
Stats record one thread, no multithreading, and `local_repair_strategy=-`.

Stats and solution CSVs are written for normal results, including failures.
Failed full-path runs also write current conflicts; a missing-path failure
without geometric conflicts has a header-only conflicts CSV. See
[CLI and result artifacts](experiment_cli_and_results.md) for schemas and
exit codes.

## Cost and verification

Full table reconstruction is limited to initialization. An attempt saves only
affected reservation entries and replaced paths, updates every removed and
inserted path, performs SIPP searches, and runs global conflict detection.
Shared cells may contain reservations from many agents, so saving their entries
can still be expensive. Incremental interval updates do not imply that the
complete attempt costs only the number of changed cells.

The tests cover instance-order priority, complete-group removal, parked owners,
multi-agent vertex/swap events, independent and overlapping conflicts,
permanent goal arrivals, failed-group rollback, reservation equivalence,
metrics, repeatability, and CLI artifacts. Run them with:

```bash
cmake --preset normal
cmake --build --preset normal --parallel
ctest --preset normal --output-on-failure
```

The [approved specification](../specs/009-full-path-repair-iterative-solver/spec.md)
defines the implementation and acceptance contracts.

Verification on 2026-10-02:

- The normal build completed and all 10 CTest targets passed, including the
  existing solvers, SIPP, reservation, CLI, and profiling-script regressions.
- The documented three-agent example exited `0`, with sum-of-costs `16`,
  makespan `6`, and stats/solution CSVs. A two-cell swap exited `1`, preserving
  its original paths and writing the remaining edge conflict.
- A temporary private selection check verified ID, time, kind, and coordinate
  ordering, the union of edge participants, and invalid-index rejection.
  Production call-site review confirmed one initial reservation build, complete
  group removal before SIPP, and insertion before each subsequent search.

TODO: Profile affected-entry copies, SIPP, and global conflict detection before
considering further optimizations or incremental collision indexes.
