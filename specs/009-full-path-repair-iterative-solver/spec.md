Created: 2026-10-02T13:51:13-03:00

Author: Lucas Martello Nogueira (local user: lucas; repository Git identity)

Last updated: 2026-10-02T13:51:13-03:00

AI model: GPT-6 (Codex)

# Specification: FullPathRepairIterativeSolver

## 1. Source, objective, and scope

Implement the approved [plan](plan.md), following [repository instructions](../../.github/ai-instructions.md) and [MAPF conventions](../../.github/mapf.md). The user approved that plan and requested this specification. The original feature request is preserved in the plan's `prompt` section; the specification request is recorded below.

Add `mapf::FullPathRepairIterativeSolver` with sequential independent A* initialization and sequential full-path SIPP repair of every participant of each selected conflict. Build reservations once, then update them through the existing incremental helpers. Within a selected conflict, remove every participant's old path before searching for the first replacement; replan participants in ascending real agent ID order and insert each replacement before searching for the next participant.

Reuse the existing contracts from [feature 005](../005-cli-results-and-iterative-local-repair/spec.md) for aligned results and experiments, [feature 007](../007-collision-index-and-local-repair-strategies/spec.md) for conflict identities and parked owners, and [feature 008](../008-incremental-path-reservations/spec.md) for reservation ownership, disposable copies, and permanent-goal SIPP. This specification defines the new solver's behavior without replacing existing solvers' repair strategies.

The implementation includes the solver, a shared metric finalizer, CLI/artifact integration, focused tests, and documentation. Use C++20, the existing standard-library containers, and CMake/CTest; add no dependencies. The approved plan contains the supporting library research.

The new solver has fixed ID priority and stops at the first unsuccessful conflict-group attempt. Local repair windows, parallel searches, configurable continuation, alternative priorities, reservation overlays, incremental collision detection, and public diagnostic counters are outside this feature. A failed prioritized attempt reports algorithmic failure, not a proof that the MAPF instance is unsolvable. Global makespan/sum-of-costs optimality and numerical speedups are not guaranteed.

## 2. Public API and result invariants

Create `include/mapf/solvers/full_path_repair_iterative_solver.hpp`:

```cpp
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

Define the constructor and `solve()` in `src/mapf/solvers/full_path_repair_iterative_solver.cpp`. The constructor stores the instance reference; initialization and repair occur in `solve()`. The instance/grid must outlive the solver and all returned non-owning cell pointers. Follow the existing solver convention for obtaining `Grid&` from the stored instance; do not change grid geometry or agent data. Each `solve()` invocation starts with fresh state.

Reuse `LocalPathRepairResult` without adding or renaming public fields:

| Field | Required meaning on every normal return |
| --- | --- |
| `initialPaths` | Independent A* paths, in instance order, preserved for the entire invocation. |
| `paths` | Last committed full path for each agent; empty only where initial A* failed. |
| `pathCosts` | Action costs of `paths`; same vector size/order, using the existing zero convention for an empty slot. |
| `initialConflicts` | `getCollision(initialPaths)`, preserved unchanged. |
| `remainingConflicts` | Conflicts in exactly the committed `paths`, including parked owners. |
| `reservations` | All four reservation views describing exactly the committed `paths`. |
| `metrics` | Computed from the committed result, with success only when every required path exists and no conflicts remain. |

All path/cost vectors must have `instance.getAgents().size()` slots. Sorting for priority must never reorder these vectors, the instance's agents, or scenario identities.

## 3. Domain, identity, and error contracts

### 3.1. Identity and time

- Conflict participant values are signed integer **path indexes** from `getCollision`. Convert valid indexes to `std::size_t` for vector access; reserve/remove using `agents[index].id`.
- Agent IDs are unique, may be negative or noncontiguous, and are ordered numerically ascending. For IDs `[42, 7]` in instance slots `[0, 1]`, repair slot `1` before slot `0`. `scenarioId` never determines priority.
- Paths contain positions at absolute integer times starting at zero, including the initial position. A nonempty path of length `n` reaches its final goal at `n - 1`; repeated cells represent waits.
- Enforce `path.size() <= SAFE_INTERVAL_INFINITY` before reservation updates. Explicit times are below the sentinel. Do not materialize infinite parked tails.
- All path cells belong to the instance's live grid. This is guaranteed by using its endpoints and the existing search implementations; arbitrary external path pointers are not a new public input surface.
- Preserve separate uniqueness of starts and goals, as already enforced by `Instance`. An agent's start may equal its own or another agent's goal.

### 3.2. Paths and conflicts

A complete path must be nonempty, contain only non-null free cells, begin/end at the agent's original start/goal, obey the supported length bound, and use only waits or cardinal neighbor moves. Implement a small private validator in the new `.cpp`; validate initial nonempty paths and each SIPP replacement before using it in reservations.

Reject vertex conflicts and swaps. Same-direction co-occupancy is already represented by vertex events. Following and otherwise collision-free cycles remain allowed. Agents occupy their final goal permanently after arrival; use `GoalOccupation::Permanent` for every full-path SIPP call.

### 3.3. Outcomes and exceptions

| Condition | Required behavior |
| --- | --- |
| Invalid CLI or instance | Existing validation reports the error before solving/artifact creation; CLI exits `2`. |
| Missing initial A* path | Finish initial searches for all agents, build the aligned initial snapshot, return unsuccessful metrics, and skip repair. |
| Malformed nonempty initial path or unsupported initial time | Throw `std::invalid_argument` before building reservations, consistent with the existing repair engine. |
| SIPP returns an empty or structurally invalid replacement | Reject the entire group and return the last committed snapshot with unsuccessful metrics. |
| A completed group's collision/progress validation fails | Reject the entire group and return the last committed snapshot with unsuccessful metrics. |
| Reservation ownership violation, allocation failure, or exception from a dependency | Propagate the exception; discard temporary state. Do not convert exceptions into ordinary no-path results or continue with a partially mutated temporary. |
| All required paths exist and no conflicts remain | Return successful metrics and the complete committed snapshot. |

Empty instances are valid and succeed with empty path/cost/conflict collections and zero costs/injustice. A one-cell path has cost zero. No-path results preserve available diagnostics; they do not remove agents or invent replacement paths.

## 4. Initial planning and single reservation build

Implement initialization in this order:

1. Record `startedAt = std::chrono::steady_clock::now()` before initial search.
2. Allocate `initialPaths` with one slot per instance agent.
3. In instance order, obtain original start/goal cells from the grid and call `AStarSolver::solve`. No reservations or other-agent paths constrain these searches. Attempt every agent even if an earlier search returns empty.
4. Check all nonempty paths against section 3.2. Track whether every initial path exists.
5. Set `paths = initialPaths`, compute `initialConflicts = getCollision(initialPaths)`, and copy that snapshot into `remainingConflicts`.
6. Call `local_path_repair_detail::buildReservationState(grid, agents, paths)` once and store its result in `reservations`.
7. If a required initial path is missing, finalize metrics with `requestedSuccess = false` and return. If conflicts are empty, finalize with `requestedSuccess = true` and return.

Every normally returning invocation, including empty instances, conflict-free instances, and missing-path results, performs exactly one full reservation build. An exception before initialization completes need not reach that call. There must be no full reservation build in repair, candidate validation, rollback, or finalization.

Do not call the existing `repairInitialPaths`: its one-agent local repair policy is a different algorithm. Reuse the A* initialization pattern and reservation functions directly.

## 5. Deterministic conflict selection

Use the current `remainingConflicts.vertexEvents` and `remainingConflicts.edgeEvents`. Keep selection helpers private to the new implementation file.

### 5.1. Group membership

- For a vertex event, include every index in `VertexEvent::participants`.
- For an active edge event, include the deduplicated union of `EdgeEvent::forward` and `EdgeEvent::reverse`.
- Include parked destination owners whose explicit path ended before the event. Their global vertex participation is authoritative even if `byAgent[owner]` has no corresponding record.
- Select the participants of that one event, not a transitive group spanning all conflicts. Events may contain more than two participants.

Use only indexes belonging to the current path snapshot. If an internally produced event contains an out-of-range participant or fewer than two distinct participants, report an internal contract error instead of silently choosing a partial group. No new detector or pairwise path rescan is needed to recover members.

### 5.2. Event and participant ordering

Select the smallest event under this lexicographic key:

```text
(minimum real participant ID, time, kind, x1, y1, x2, y2)
```

`kind` is `0` for vertex and `1` for edge. Repeat the vertex's coordinates for both endpoints. For edges, use the detector's canonical coordinate-ordered endpoints. Compare signed IDs directly rather than subtracting them. Do not order by hash iteration, pointer addresses, path indexes, or scenario buckets.

The selected value contains its event key and a `std::vector<std::size_t>` of distinct participant indexes, sorted by `agents[index].id`. A private `std::variant<CellTime, EdgeTime>` is suitable for the key. Copy these values; do not retain references or iterators into a conflict snapshot across publication.

Recompute selection after every successful group commit. An initially nonempty conflict snapshot must yield an event; failure to do so is an internal contract error. No queue of stale events or revision-based retry policy is needed.

## 6. Complete-group repair transaction

### 6.1. Copy and remove

For each selected event, copy `result.paths` once into `candidatePaths` and `result.reservations` once into `candidateReservations`. Do not create another full reservation copy per participant.

Before calling SIPP for any participant, loop over the entire selected group and execute:

```cpp
local_path_repair_detail::repairSafeIntervalTable(
    candidateReservations, result.paths[index], agents[index].id);
```

Pass the registered complete old path with absolute time starting at zero. Remove each participant exactly once. The state after this loop must represent exactly the committed paths of nonparticipants.

The helper's existing ownership behavior must remain intact: removal updates explicit vertex ownership, permanent goal occupancy, and reverse-edge arrival blocks without removing contributions from other agents. Preserve other owners sharing a cell/time or movement/time. Do not clear whole reservation entries manually.

### 6.2. Replan and insert

Traverse the participants in their sorted real-ID order. For each participant:

1. Read its original start and goal from the instance, not from a local repair window or current position.
2. Call the existing supplied-table SIPP overload at time zero with permanent goal occupation.
3. Validate the returned complete path. An empty/invalid path rejects the group immediately; later participants are not searched.
4. Call `updateReservationState` with that new complete path and the real ID.
5. Move the replacement into its original slot in `candidatePaths`.
6. Only then begin the next participant's search.

The essential call sequence is:

```cpp
// All old group reservations have already been removed.
auto replacement = sipp.solve(
    grid, start, goal, candidateReservations.safeIntervalTable,
    0, AStarSippSolver::GoalOccupation::Permanent);

// Reject the group here if replacement is empty or structurally invalid.
local_path_repair_detail::updateReservationState(
    candidateReservations, replacement, agents[index].id);
candidatePaths[index] = std::move(replacement);
```

The safe table before participant `j` searches must contain exactly the unchanged nonparticipant paths and the new paths of participants preceding `j`. Later group members have no reservation contribution. Although their old paths may still occupy temporary vector slots, never derive the search table from that vector or use those stale slots as obstacles.

Every participant must be searched during a successful group attempt, even when an earlier search already changes the selected collision or returns a path identical to its old path. Such an unchanged replacement still requires insertion before proceeding. No suffix reuse, local bridge search, or per-agent partial commit is allowed.

Only the six-argument supplied-table SIPP overload is permitted here. The five-argument table overload uses transient goal occupation, and the path-vector overload builds reservations internally; neither implements this contract. Do not call `getSafeIntervalsByCell` during repair. Keep the table immutable for the duration of each search because SIPP states hold interval indexes.

### 6.3. Validate the complete candidate

After all participant paths have been inserted, call `getCollision(candidatePaths)` once to obtain `candidateConflicts`. Do not validate an incomplete group against the old paths of participants that have not yet been replanned.

A group can commit only if both conditions hold:

1. No event in `candidateConflicts.vertexEvents` or `candidateConflicts.edgeEvents` contains a selected participant. Scan global participant sets, including both edge directions and parked owners. A check of `byAgent` alone is insufficient.
2. The total number of global vertex plus edge events is strictly smaller than in `result.remainingConflicts` before this attempt. Count event-map entries, not per-agent records or participant pairs.

Remaining conflicts solely among nonparticipants do not reject the group. Their paths have not changed and they can be handled by subsequent iterations. These checks do not rebuild reservations.

### 6.4. Publish or discard

On successful validation, publish the complete snapshot using non-throwing moves:

```cpp
static_assert(std::is_nothrow_move_assignable_v<decltype(result.paths)>);
static_assert(std::is_nothrow_move_assignable_v<SolutionConflicts>);
static_assert(std::is_nothrow_move_assignable_v<PathReservationState>);

result.paths = std::move(candidatePaths);
result.remainingConflicts = std::move(candidateConflicts);
result.reservations = std::move(candidateReservations);
```

Include `<type_traits>` for these checks. Finish all allocating work and candidate validation before the first assignment. There must be no potentially throwing work between these three assignments for the existing container/allocator types. `initialPaths` and `initialConflicts` remain untouched.

On an unsuccessful search or failed candidate check, discard both candidate objects, finalize the committed result with `requestedSuccess = false`, and return. Do not add old paths back, rebuild a table to restore it, retain some replacements, clear failed slots, or try a different event. Earlier committed groups stay in the result; only the current uncommitted attempt is discarded.

If an incremental helper throws after changing its temporary state, discard that state and propagate the exception. The helpers do not promise rollback of their disposable argument.

### 6.5. Continue and terminate

After publication, repeat selection from the new conflict snapshot. When it is empty, finalize metrics with `requestedSuccess = true` and return.

The progress check supplies the termination condition. Replanned agents are conflict-free against all outsiders and each other, outsiders' paths are unchanged, and the selected event disappears. Thus every commit strictly reduces the finite number of events; every failed attempt returns. Do not add repeated retries, arbitrary iteration caps, or the local engine's configuration-fingerprint machinery.

## 7. Shared metric finalizer

In `src/mapf/solvers/local_path_repair_solver_common.hpp`, declare the existing finalizer inside `mapf::local_path_repair_detail`:

```cpp
void updateResultMetrics(
    LocalPathRepairResult& result,
    const std::vector<Agent>& agents,
    bool requestedSuccess,
    std::chrono::steady_clock::time_point startedAt);
```

Move its definition out of the anonymous namespace into that internal namespace in the corresponding `.cpp`. Keep the private `pathCost`, `calculateInjustice`, and duration helpers private and reuse them. Preserve the finalizer's implementation and existing local-engine caller behavior; avoid a second metric implementation in the new solver.

Call the finalizer on every normal solver return:

- `pathCosts[i]` is zero for an empty path or `paths[i].size() - 1` otherwise.
- `sumOfCosts` sums those action costs; `makespan` is their maximum or zero for no paths.
- `injustice` is the population standard deviation of final-minus-initial action costs over slots where both paths exist; use zero if none exist.
- `durationSeconds` includes initial A*, conflict detection, reservation building/copying/updates, and repair up to finalization. CSV serialization is outside solver timing.
- `success` is `requestedSuccess && allPathsExist && remainingConflicts.empty()`. An empty instance satisfies path completeness.

The artifact writer continues to display a missing path's per-agent cost as `-1` while excluding missing paths from aggregate costs. Do not change either convention.

## 8. CLI and artifact integration

### 8.1. Parsing and dispatch

In `src/main.cpp`, include the new public header, list `FullPathRepairIterativeSolver` in usage, and recognize that exact case-sensitive solver name during validation and dispatch.

Required flags remain `-map`, `-scen`, `-solver`, and `-agents`; retain all current duplicate-flag, numeric, path, and scenario validation. For this solver, reject explicitly supplied `-threads`, `-continue_if_failed`, and `-localRepairStrategy`, including values `1`, `false`, or `RESOLVE_BY_AGENT` that resemble defaults. Check the existing presence markers rather than only the effective values.

Dispatch with `FullPathRepairIterativeSolver solver(instance)` and `LocalPathRepairResult result = solver.solve()`. Fill the experiment result as follows:

| `ExperimentRunResult` field | Required value |
| --- | --- |
| `metrics` | `result.metrics` |
| `initialPaths` | Move from `result.initialPaths`. |
| `solutionPaths` | Move from `result.paths`. |
| `numAgents` | The requested/loaded agent count, as for existing solvers. |
| `solver` | `"FullPathRepairIterativeSolver"` |
| `continueIfFailed` | `false` |
| `multithreading` | `false` |
| `numThreads` | `1` |
| `localRepair` | `false` |
| `localRepairStrategy` | `std::nullopt` |

Keep experiment duration boundaries and the existing writer/error handling. Existing solver names, options, and dispatch behavior must remain unchanged.

### 8.2. Writer metadata and failure artifacts

In `src/mapf/experiments/experiment_utils.cpp`:

1. Add the exact new name to `supportedSolver`.
2. Extend `validSolverMetadata` to require all six configuration values in the table above (`solver` through `localRepairStrategy`) for the new solver. In particular, reject an engaged local strategy, `localRepair == true`, continuation, or multithreading. Preserve checks for existing solvers.
3. Replace the conflict-artifact predicate with the equivalent of:

```cpp
const bool needsConflicts = !run.metrics.success &&
    (run.localRepair || run.solver == "FullPathRepairIterativeSolver");
```

Continue using `normalizeConflicts` on final paths to obtain real IDs and deterministic conflict rows. Retain original initial paths in `optimum_path`, committed final paths in `solution_path`, and repeated cells for waits. Do not serialize discarded candidates.

Preserve the existing output directory, atomic bundle writing, CSV headers, and per-agent success calculation. The new solver records `false` in `continue_if_failed` and `multithreading`, `1` in `num_threads`, its exact class name in `solver`, and `-` in `local_repair_strategy`.

Stats and solution CSVs are written for every normal result. An unsuccessful full-path result also writes the conflicts CSV; if failure is due only to missing paths, that CSV may contain just its header. Successful full-path runs have no conflicts CSV. Existing local-repair and priority-planning artifact conditions remain unchanged.

### 8.3. Exit codes

| Code | Meaning |
| --- | --- |
| `0` | Solver succeeded and the complete artifact bundle was written. |
| `1` | Solver returned an unsuccessful result, or artifact writing failed. Algorithmic failure still attempts to write diagnostics. |
| `2` | Invalid CLI/input or a solver exception before a result is available; no experiment bundle is created. |

## 9. Implementation files and required order

| Step | Files | Required change |
| --- | --- | --- |
| 1 | `src/mapf/solvers/local_path_repair_solver_common.hpp`, `src/mapf/solvers/local_path_repair_solver_common.cpp` | Expose the existing metric finalizer through the internal namespace while preserving existing callers and calculations. |
| 2 | `include/mapf/solvers/full_path_repair_iterative_solver.hpp` (new) | Public class from section 2. |
| 3 | `src/mapf/solvers/full_path_repair_iterative_solver.cpp` (new) | Private path validation/event-selection helpers, sequential initial planning, group transactions, progress checks, and result finalization. |
| 4 | `CMakeLists.txt` | Add the new solver source to `mapf` and register the solver test target under `BUILD_TESTING`. |
| 5 | `src/main.cpp` | Usage, option validation, dispatch, and experiment metadata. |
| 6 | `src/mapf/experiments/experiment_utils.cpp` | Solver allowlist, strict metadata validation, and unsuccessful full-path conflict bundles. |
| 7 | `tests/solvers/full_path_repair_iterative_solver_test.cpp` (new), `tests/solvers/path_reservation_state_test.cpp` | Behavioral solver tests and complete-group reservation equivalence cases. |
| 8 | `tests/experiments/cli_test.cpp`, `tests/experiments/experiment_utils_test.cpp` | Dispatch, rejected flags, metadata validation, artifacts, and compatibility checks. |
| 9 | `docs/full_path_repair_iterative_solver.md` (new), `docs/experiment_cli_and_results.md`, `readme.md` | Delivered algorithm, options, result semantics, and experiment examples. |

Register `full_path_repair_iterative_solver_test` as a standalone executable linked to `mapf` and as a CTest test of the same name. Give it private include access to `tests` and `src/mapf/solvers` so it can use `test_support.hpp` and the reservation builder as an oracle. Follow the existing `path_reservation_state_test` pattern; do not add a test framework or expose private selection helpers through a new public API.

Reuse the reservation helper implementations, A*/SIPP APIs, conflict detector, public result/reservation structures, `Instance` validation, and experiment result structure unchanged. Preserve existing solver APIs, presets, scripts, examples' solver choices, and CSV schemas. The new solver must not call or alter the local engine's repair loop.

## 10. Required verification

### 10.1. Solver behavior

Use `tests/test_support.hpp`. Prefer assertions about path validity, exact simple-fixture behavior, preserved snapshots, and reservation contents over tests that duplicate the implementation. Compare coordinates when separate instances own different grids. Do not require equality between the full-path and local-repair solutions.

| Case | Acceptance assertion |
| --- | --- |
| Empty instance, one agent, start equals goal | Successful aligned results; zero costs for empty/unit paths; correct permanent goal reservation for a unit path. |
| Initially conflict-free paths | Final paths equal independent A* paths; no SIPP search; no remaining events. |
| Unreachable first agent and reachable later agent | Later A* still runs; slots remain aligned; missing initial slot stays empty; no repair; unsuccessful result and correct metrics. |
| Two-agent perpendicular crossing on an open `3 x 3` grid, IDs `[42, 7]` in that order | Both initial paths conflict at the center. The smaller ID's replacement has its unconstrained shortest cost because both old paths were removed; the later participant avoids its inserted replacement. Final paths are complete and conflict-free. |
| Negative/noncontiguous IDs and nonunique scenario buckets | Priority uses signed real IDs, reservations store those IDs, and paths retain their instance slots. |
| Three or more participants sharing one vertex | All selected participants are removed and searched; no participant is lost by reducing an event to a pair. |
| Edge swap, including events with multiple participants in direction sets | Membership includes both directions and is deduplicated; reverse-edge reservations prevent swaps. |
| Collision with a parked goal owner beyond its finite path | Global participation includes the parked owner even without a `byAgent` record for that time; candidate validation also checks global events. |
| Goal safe temporarily before a future visit | Permanent-goal SIPP waits/detours to a permanent safe arrival or returns failure; it never accepts parking in a finite safe interval. |
| Independent conflict groups | Each commit is selected from refreshed conflicts; more than one group can be resolved without a rebuild. |
| Overlapping events and unaffected agents | Every selected member is replanned against outsiders; untouched paths stay unchanged; surviving outsider events are handled later. |
| Impossible swap in a two-cell corridor | The first participant's new path can be inserted in the temporary but the second fails. Returned paths/conflicts/reservations equal the pre-attempt snapshot, apart from finalized metrics/duration. |
| Successful group before an independently failing group | Use lower IDs for the successful group to establish selection order. Its committed repair remains; the failing group's tentative changes are discarded. Initial diagnostics stay unchanged. |
| Following moves and collision-free cycles | Accepted according to the existing MAPF convention, without extra conflict types. |
| Repeated invocation on the same instance | Same paths, conflicts, and costs; allow elapsed time to differ. No reservations leak across invocations. |
| Invalid starts/goals | Existing `Instance` constructors reject duplicate starts or duplicate goals before solving. |

On successful results, independently check original endpoints, legal steps, nonempty required paths, `getCollision(paths).empty()`, and metric values. On unsuccessful normal returns, check that `remainingConflicts` matches fresh detection and reservations match the returned paths. A collision-free detector result alone must not turn missing-path failure into success.

Test event-priority ties using fixtures whose observable paths establish the expected minimum-ID/time/kind/coordinate ordering. Where operation counts cannot be established by outputs, use the review/instrumentation check in section 10.4; do not add public knobs merely to observe private calls.

### 10.2. Reservation equivalence and ownership

Extend the existing reservation tests to cover all removals of a conflict group followed by sequential insertion of complete replacement paths.

For a full-build reference, retain the original vector size/order and clear the slots of excluded participants. Compare the incremental temporary with `buildReservationState(grid, agents, referencePaths)`:

1. After removing every old participant path.
2. After each new participant path is inserted, with only already-inserted replacements present in the reference group slots.
3. After a successful group commit and on a solver's failed normal return.

Compare semantic contents of all four views: `vertex_agents`, `goal_reservations` including owner/arrival, safe interval endpoints, and blocked edge arrival sets. Hash iteration order is irrelevant. Full reconstruction is permitted here as a test oracle, never as a production repair operation.

Include explicit expected outcomes for shared cell/time occupancy, shared movement/time contributors, opposite edge directions, waits, revisits, removed permanent owners with remaining visitors, and changed goal-arrival times. Verify that obsolete path contributions disappear while unaffected reservations survive. Keep checks against direct occupancy/known intervals so shared builder logic cannot hide an error.

### 10.3. CLI and artifact checks

Add success, algorithmic-failure, missing-initial-path, and zero-agent runs for the exact new solver name. Verify:

- Each prohibited flag is rejected even when its supplied value resembles a default, and invalid input creates no bundle.
- Metadata matches section 8; manually constructed inconsistent writer metadata is rejected.
- CSVs preserve the existing headers and instance row order. Initial/final paths, waits, action costs, real IDs, scenario buckets, aggregate metrics, and `local_repair_strategy=-` are correct.
- A successful run writes stats and solution only. Failed runs also write current conflicts, including a header-only file for a failure without geometric events.
- Failed-group temporary paths never reach output, while repairs from earlier committed groups do.
- Return codes remain `0`, `1`, and `2` under the conditions in section 8.3.
- Existing priority-planning and both local-repair solvers retain their options, results, and artifact behavior.

### 10.4. Reservation-work and regression checks

Review production call sites and use temporary private instrumentation if needed to verify:

- Exactly one call to `buildReservationState` in every normal full-solver invocation.
- One reservation-state copy per selected group, not per participant.
- All group removals finish before the first SIPP call.
- Each successful participant search is followed by insertion before the next search.
- No `getSafeIntervalsByCell`, path-vector SIPP overload, or full-build rollback occurs during repair.
- All published groups pass participant-freedom checks and strictly reduce global event count.

Run the full build and test suite after implementation, including local/parallel solver regressions affected by the shared metric finalizer:

```bash
cmake --preset normal
cmake --build --preset normal --parallel
ctest --preset normal --output-on-failure
```

Run this CLI smoke example and inspect its artifact bundle; algorithmic success depends on the selected scenario, so use the deterministic fixtures above for success/failure assertions:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver FullPathRepairIterativeSolver \
  -agents 3
```

Report checks actually executed. Distinguish runtime costs of copying existing state from reconstructing reservations; a reduction in full builds alone does not demonstrate a numerical speedup.

## 11. Documentation and completion criteria

`docs/full_path_repair_iterative_solver.md` must explain independent initialization, global-event membership, real-ID priority, removal of the complete group before searching, permanent-goal SIPP, sequential insertion, atomic publication, failure/termination behavior, metric conventions, and complexity limits from state copying and global collision detection.

Update `readme.md` and `docs/experiment_cli_and_results.md` with the new solver and example, rejected flags, `local_repair_strategy=-`, and failure artifacts. Align their solver lists and stats-schema descriptions with the actual writer. When touching the existing strategy description, correct the README's stale claim that `RESOLVE_BY_TIME` is unimplemented; its current common-engine implementation is already present.

Implementation is complete when all of the following hold:

1. The new public class and CLI name build and execute with the specified result/metadata contracts.
2. Initial A* attempts cover all agents and preserve their original aligned diagnostics.
3. Every successful group includes all event participants, removes all old contributions first, and replans full paths in ascending real ID order using the permanent supplied-table SIPP overload.
4. Reservations use one initial build and incremental removal/insertion thereafter; committed snapshots remain consistent on both success and failure.
5. No repaired participant remains in a global conflict after a commit, every commit makes measurable event-count progress, and no-path results stop without publishing a partial group.
6. CLI artifacts and return codes follow section 8, existing solvers retain their behavior, and the required tests/regressions pass.
7. The documentation describes the delivered behavior and reports verification without claiming global completeness, optimality, or an unmeasured speedup.

## 12. Deferred work

- TODO: Profile complete state/path copies, SIPP, and global detection before introducing overlays or incremental collision indexes.
- TODO: Consider a neutral shared result/reservation module if additional full-path solvers make the current local-repair type naming confusing.
- TODO: Specify continuation and alternate-priority policies separately if requested; both require explicit group rollback and scheduling rules.

## prompt

```text
aprovado, agora faça o arquivo spec
```
