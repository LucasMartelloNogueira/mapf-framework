Created: 2026-09-29T13:45:30-03:00

Author: Lucas Martello Nogueira (local user: lucas; repository Git identity)

Last updated: 2026-09-29T14:42:09-03:00

AI model: GPT-6 (Codex)

# Specification: Incremental Path Reservations

## 1. Source, objective, and scope

Implement the approved [plan](plan.md), following [repository instructions](../../.github/ai-instructions.md) and [MAPF constraints](../../.github/mapf.md).

The [consolidated documentation](../../docs/incremental_path_reservatons.md) is the reference for the original problem, user decisions, correctness examples, performance assessment, and deferred alternatives. It incorporates the initial review, user responses, follow-up review, and original plan. The implementation contracts and acceptance criteria remain defined in this specification.

The objective is to eliminate complete reservation reconstruction inside the local repair loop and its full-path fallback. Keep one initial full build and one copied `PathReservationState` per actual repair attempt. Remove the active agent from that copy, search against it, and add the accepted replacement before committing.

This feature replaces the reservation rebuilding requirements of previous local-repair specifications. The approved plan explicitly includes migration and addition of tests. Preserve the remaining behavior of [feature 007](../007-collision-index-and-local-repair-strategies/spec.md): conflict selection, bounded local repair, candidate evaluation, cycle protection, continuation, and consistent publication. Both iterative and parallel solvers use this same sequential engine.

Use C++20, existing standard-library containers, and CMake/CTest. Keep the current `SafeIntervalTable` layout. Overlays, incremental conflict detection, additional runtime dependencies, new CLI flags, and parallel reservation updates are outside this implementation.

## 2. Data model and invariants

In `include/mapf/solvers/local_path_repair_solver.hpp`, define:

```cpp
using VertexOccupants = std::unordered_map<Cell*,
    std::unordered_map<int, std::unordered_set<int>>>;

struct GoalReservation {
    int agentId;
    int arrivalTime;
};

struct PathReservationState {
    SafeIntervalTable safeIntervalTable;
    VertexOccupants vertex_agents;
    std::unordered_map<Cell*, GoalReservation> goal_reservations;
};
```

The following rules are mandatory:

| Component | Meaning and invariant |
| --- | --- |
| `vertex_agents[cell][t]` | Real `Agent::id` values explicitly occupying the cell at path index `t`. Include the last path element. Store only nonempty sets and nonempty per-cell maps. |
| `goal_reservations[cell]` | One real agent ID and its arrival time, representing permanent occupancy from that time onward. Goals remain unique. |
| `safeIntervalsByCell[cell]` | Sorted maximal closed safe intervals, with no overlap or adjacent intervals left unmerged. Include every grid cell as in the initial builder. Fully free means `[0, SAFE_INTERVAL_INFINITY]`; fully blocked means an empty vector. |
| `blockedEdgeArrivals[{V,U}]` | Arrival times of explicit movements `U -> V`, where `U != V`. Store only nonempty sets. |

An ID is not a path index: ID `42` may occupy slot `0`. Reserve using `agents[activeIndex].id`; continue using path indexes in `SolutionConflicts`. Do not reuse or change the detector's private `GoalOccupancy` in `utils.cpp`.

Paths use absolute time starting at zero. A nonempty path of length `n` arrives at `n-1`. Explicit times must remain below `SAFE_INTERVAL_INFINITY`, so preserve the existing maximum supported length of `SAFE_INTERVAL_INFINITY`. The constant is a sentinel, never a loop bound for materializing permanent occupancy.

Occupied times are the union of explicit visits and permanent goal occupancy. The final explicit visit overlaps the permanent interval intentionally; union semantics must prevent either omission or double-removal. Waits and revisits remain distinct explicit timesteps. Cells are non-owning pointers into the same live grid; operations must not mutate grid geometry or obstacle data.

The state may contain collisions among different agents: this is normal before repair. Unique origins/goals remain enforced by the existing instance and path validation. Following and cycle conflicts remain allowed.

## 3. Initial construction and cell regeneration

Retain the initial `buildReservationState(grid, agents, result.paths)` in `repairInitialPaths`. Populate all four components using the new representation. Empty path slots contribute nothing, and existing empty-instance/unreachable-path behavior is preserved.

Keep the optional excluded-path-index form of the full builder as a test/reference operation. It must omit all contributions of that slot. Initial construction shall register included paths first, then generate each grid cell's safe intervals once; do not repeatedly regenerate cells while registering each path.

Extract a cell-local regeneration helper with this algorithm:

1. Collect `[t,t]` for each nonempty explicit owner set of the cell, plus `[arrivalTime, SAFE_INTERVAL_INFINITY]` if it has a goal reservation.
2. Sort and merge overlapping or adjacent blocked intervals using the existing interval conventions.
3. Compute their complement from zero: emit `[safeStart, blocked.start-1]` only when nonempty; stop at a permanent blocked tail; otherwise advance past the finite blocked end.
4. Write the normalized vector for this cell, retaining a fully free entry when its last occupant disappears.

Never assume the temporal hash map is ordered. Guard time conversions and sentinel arithmetic. Do not erase the safe-table entry to represent freedom: existing SIPP lookups treat a missing entry as unavailable.

Examples, assuming no other occupancy:

| Change | Correct outcome |
| --- | --- |
| Remove the only block at 3 between `[0,2]` and `[4,+inf]`. | One safe interval `[0,+inf]`. |
| Remove occupancy at 6 from blocked `[5,7]`. | New safe interval `[6,6]` between the remaining blocks. |
| Add a visit at 5 to a fully free cell. | Safe intervals `[0,4]` and `[6,+inf]`. |
| Remove a goal owner parked since 4 while another agent visits at 7. | Safe intervals `[0,6]` and `[8,+inf]`. |
| Remove a visitor at 5 while another agent has been parked since 3. | Time 5 remains blocked by the goal reservation. |

## 4. Reservation helper contracts

Declare the full builder and mutation helpers in `src/mapf/solvers/local_path_repair_solver_common.hpp`, inside `mapf::local_path_repair_detail`, for use by the shared engine and focused tests. Define them in the corresponding `.cpp`; keep implementation-only utilities private.

```cpp
void repairSafeIntervalTable(
    PathReservationState& state,
    const std::list<Cell*>& oldPath,
    int agentId);

void updateReservationState(
    PathReservationState& state,
    const std::list<Cell*>& newPath,
    int agentId);
```

Both mutate the supplied state and must not copy it internally. The caller supplies a disposable copy of the committed state. Full-path validity and endpoint alignment remain checked by the existing engine. Empty paths are no-ops; they must not access `front()` or `back()`.

For nonempty paths, reject null cells or unsupported lengths before mutation. Removal requires the registered old path for this ID; insertion requires that its previous contribution has already been removed. Check expected explicit memberships and the old goal owner/arrival before removal, and reject an occupied destination entry before insertion. Ownership violations must report `std::logic_error`; malformed path values must report `std::invalid_argument`. These checks must not scan unrelated cells or paths. Exact registered-path correspondence is an internal caller precondition; do not add a global agent-to-path registry merely to revalidate it.

The helpers do not promise rollback of their disposable argument after an allocation failure. Propagate the exception and discard that argument. Never mutate the committed result directly or continue searching with a partially updated temporary. Ordinary search failure remains separate from an exception.

### 4.1. `repairSafeIntervalTable`: remove one path

1. Collect the old path's distinct cells and its actual movements with absolute arrival times.
2. Remove only `agentId` from each explicit cell/time set. Erase empty sets and empty occupancy maps. Do not remove another agent or another visit time.
3. Remove the matching permanent goal entry, including the entire tail from its arrival time.
4. After all explicit removals, update only the reverse-edge/time entries contributed by old movements using the rule in section 4.3.
5. Regenerate safe intervals once for each distinct old-path cell, including its goal.

Removing one of several occupants may leave the safe intervals unchanged, but the owner sets must still change. Repeating removal of the same nonempty path is a contract error, not another decrement.

### 4.2. `updateReservationState`: add one replacement

1. Add `agentId` at every explicit timestep of the new complete path.
2. Register `{agentId, newPath.size()-1}` at its last cell.
3. Insert the reverse-edge arrival for each non-wait movement.
4. Regenerate safe intervals once for each distinct new-path cell.

Insert into the already-excluded temporary, never into the original state that still contains the old path. This covers new cells and obsolete cells through the two operations' combined affected sets.

Use the entire accepted path, not only the SIPP bridge. For example, two additional waits move a later reservation from `B@8` to `B@10`, even if the suffix geometry is unchanged. Complete old removal and new insertion must update that timestamp and the permanent goal arrival.

### 4.3. Preserve shared edge reservations

For an old movement `U -> V` arriving at `t`, check the remaining explicit owner sets at `(U,t-1)` and `(V,t)` after removal. Keep `{V,U},t` blocked exactly when their intersection is nonempty. Otherwise erase that time and, if empty, its edge map entry. Query with `find`, avoiding accidental empty occupancy entries.

This check is exact because each registered agent has one explicit position per timestep. Parked tails produce no new movements and need not be expanded. Iterate the smaller owner set when checking intersection. Waits `U == V` never register an edge.

Example: X and Y both traverse `U -> V` at time 5. Removing X preserves the reverse block because Y is in both remaining endpoint sets. Removing the last contributor removes that block. A block in the opposite direction or at a different time remains untouched.

## 5. SIPP API and spatial queries

Make `AStarSippSolver::GoalOccupation` public and add this explicit-policy overload in `a_star_sipp.hpp/.cpp`:

```cpp
std::list<Cell*> solve(
    Grid& grid, Cell* start, Cell* goal,
    const SafeIntervalTable& safeIntervalTable,
    int startTime, GoalOccupation goalOccupation);
```

Delegate to the existing `solveWithTable`; it must not build a table or copy other agents' paths. Keep the existing five-argument table overload transient and the path-vector overload permanent. Preserve SIPP's movement order, tie-breaking, time-offset reconstruction, and goal-policy checks. Reject unsupported enum values rather than silently choosing a policy.

All local bridges and existing suffix mini-searches retain transient semantics and existing final-goal validation. Only full fallback changes its call site to use the supplied table, `startTime = 0`, and `GoalOccupation::Permanent`.

If the real goal is safe during `[0,4]` and `[6,+inf]`, transient search can arrive at 3. A complete path may park only after arrival in the infinite interval; permanent search must find arrival at 6 or later if a safe route exists. Post-search rejection of an early transient solution is not an equivalent fallback.

Update `hasOtherAgent` to inspect all explicit timestep sets for any ID different from the active one, with early exit. Its consumers, `buildLocalCandidate` and `appendScenarioTwoPointThreeSuffix`, continue using the committed spatial context. Explicit final-cell membership ensures a parked owner remains represented spatially without expanding its tail. Time-specific safety must still consult safe intervals and edge blocks.

## 6. Repair-loop integration and publication

Apply the following sequence to `repairInitialPaths` for both solver adapters:

1. Build the initial complete state once, before the loop, and preserve existing early returns.
2. After selection and the existing `firstAttempt` check, copy `result.reservations` once. Remove `result.paths[activeIndex]` with `agents[activeIndex].id`. Do not create a copy for a skipped repeated attempt.
3. Use that same excluded table for all adaptive anchors, suffix checks, and full fallback. Remove the local engine's `pathsExcept` call and helper once unused.
4. Keep the entire temporary stable while SIPP or interval readers use it. SIPP nodes hold interval indexes; splitting or merging those intervals during a search would invalidate their meaning.
5. Preserve all `evaluateCandidate` checks and its complete provisional paths/conflicts. Rejected local candidates must not add reservations; fallback still uses the same excluded state.
6. If no candidate is accepted, discard the temporary and execute the existing failure/continuation branch. Committed paths, conflicts and reservations remain consistent.
7. If accepted, add `accepted->paths[activeIndex]` to the excluded temporary. Finish all allocating work, fingerprint-history updates and overflow checks before publishing.
8. Publish paths, conflicts, reservations and configuration using non-throwing moves/swaps for the existing allocator/container types; then retain the current revision increment and cursor/ignored-conflict reset. Verify that publication cannot leave only some result components updated.

The temporary must survive the current `if (firstAttempt)` block and be mutable between searches. One implementation is an outer `std::optional<PathReservationState>` populated with `emplace(result.reservations)`, then moved into the result on acceptance. This must not introduce a second full copy.

```text
committed state
    -> copy -> remove old full path -> search and evaluate
        -> rejected: discard copy
        -> accepted: add new full path -> publish complete configuration
```

Keep initial diagnostics, cycle history, conflict ordering, metrics other than elapsed time, CLI output, and both selection strategies unchanged. No cache may carry an excluded state into another revision. A successful solution must still pass the existing collision validation; partial results must describe the last committed configuration.

## 7. Implementation files and order

| Step | Files | Deliverable |
| --- | --- | --- |
| 1 | `include/mapf/solvers/local_path_repair_solver.hpp` | New temporal and goal-owner types. |
| 2 | `src/mapf/solvers/local_path_repair_solver_common.hpp/.cpp` | Adapted full builder, cell regeneration, validated removal/insertion, shared-edge handling and spatial query migration. |
| 3 | `include/mapf/pathfinding/a_star_sipp.hpp`, `src/mapf/pathfinding/a_star_sipp.cpp` | Public goal policy and supplied-table overload. |
| 4 | `src/mapf/solvers/local_path_repair_solver_common.cpp` | One-copy loop integration and removal of obsolete full rebuilds/path copying. |
| 5 | `tests/solvers/path_reservation_state_test.cpp` (new), `CMakeLists.txt` | Focused tests linked to `mapf`, with private access to the common-engine header and a CTest registration. |
| 6 | Existing SIPP and iterative/parallel local-repair test files | Migrated assertions and search/transaction regressions. |
| 7 | `docs/local_path_repair_parallel_solver.md` | Updated representations, lifecycle, goal-policy API, ownership, and performance limits. |
| 8 | `docs/incremental_path_reservatons.md` | Consolidated problem analysis, accepted approaches, examples, validation evidence, and deferred improvements; linked by this specification and the plan. |

Audit all occurrences of `vertex_agents` and `goal_reservations`. In particular, old tests such as `vertex_agents.at(cell).contains(0)` still compile but now test time zero, not ID zero; rewrite them to inspect owner sets. Goal assertions must compare both owner identity where relevant and `arrivalTime`.

## 8. Verification and acceptance

### 8.1. Equivalence and expected outcomes

For valid inputs, compare all four views against the full builder:

```text
copy(fullState) - oldPath[i]
    == buildReservationState(grid, agents, paths, i)

(copy(fullState) - oldPath[i]) + newPath[i]
    == buildReservationState(grid, agents, updatedPaths)
```

Compare semantic contents rather than hash iteration order. Compare interval endpoints explicitly and goal records by both fields. Include hand-written expected outcomes as well as rebuild comparisons so shared helper code cannot conceal an error. Exercise multiple consecutive substitutions to detect stale state.

| Required case | Acceptance condition |
| --- | --- |
| Shared cell/time; different visit times | Remove only the selected owner/time, preserving remaining blocks. |
| Waits, revisits, one-cell path | Correct temporal sets; no wait edges; permanent reservation from zero for a unit path. |
| Empty slots and empty instances | No reservations for empty paths; existing solver result semantics preserved. |
| Interval boundary/interior changes | Correct extension, splitting, singleton creation and merging with inclusive endpoints. |
| Parked owner removed while visitors remain | Preserve later explicit visits and free only genuinely unoccupied times. |
| Visitor removed from another owner's goal | Preserve the permanent block even without an explicit visit at that time. |
| Shared movement, opposite direction, different arrival time | Preserve remaining contributors and unrelated edge entries; remove only the last contributor's block. |
| Detour and earlier/later replacement | Remove obsolete cells/edges and register all new cells, shifted suffix times and goal arrival. |
| IDs such as 10 and 42 in slots 0 and 1 | Both vertex and goal records use real IDs; conflicts remain indexed by slot. |
| Removal with wrong owner, repeat removal, duplicate goal insertion | Report contract errors without silently corrupting ownership. |
| Supplied-table permanent fallback | Find a later permanent arrival when the earliest transient interval is insufficient; preserve old overload behavior. |
| Rejection and continuation | Failed local/fallback attempts preserve committed state and later repairs still see all other agents. |
| Untouched cells and edges | Contents remain unchanged; regeneration never visits unrelated cells. |
| Both strategies and both adapters | Valid solutions, unchanged deterministic behavior for identical inputs/strategy, and matching reservation states. |
| Supported temporal boundaries | Checked conversions and safe sentinel handling without allocating paths proportional to infinity. |

Register `path_reservation_state_test` following the existing standalone test style. Its private include directories must include `tests` and `src/mapf/solvers`. Rebuild production targets and run the complete CTest suite, including priority planning and CLI regressions affected by the public SIPP overload.

### 8.2. Work reduction and performance evidence

On production local-repair execution, require exactly one initial complete reservation build, no later full build in the loop or its fallback, and one complete state copy per actual attempt. Each removal/addition must regenerate a cell at most once per operation and only when it belongs to that operation's path. Movement checks may consult other owners at affected endpoint/times.

Confirm these properties through code inspection and private measurement/test instrumentation as needed. Equal output alone does not prove reduced work. Do not add a public logging interface merely for this check.

Compare identical optimized workloads before and after implementation: same instance, strategy, thread count, compiler configuration, and repetition procedure. Record initial construction, copying, removal, insertion, SIPP and conflict-detection time, total solve time, cell-regeneration counts, and memory. Report actual results and limitations; no numerical speedup is required or assumed. Costs still include a full copy and the other occupants of affected cells.

The feature is complete when the required tests pass, documented ownership and transaction invariants hold, no full reservation rebuild remains in local attempts/fallback, and the measurements and documentation describe the delivered behavior.

## 9. Deferred work

- TODO: Replace copied reservation states with an overlay after this version is validated and measured.
- TODO: Add a spatial membership cache only if temporal scanning becomes significant.
- TODO: Consider sharing interval construction with the standalone SIPP builder after this migration.
- TODO: Profile candidate path copying, fingerprints and global conflict rebuilding before expanding incremental updates to them.

## prompt

```text
aprovado, agora faça o arquivo spec do plano specs/008-incremental-path-reservations/plan.md
```

## adjusments

### adjustment 1

Datetime: 2026-09-29T14:42:09-03:00

Full user prompt:

```text
junte os seguintes documentos: melhoria_3.md, melhoria_3_resposta.md, melhoria_3_1.md e plano_melhoria_3.md em um unico arquivo em docs/incremental_path_reservatons.md, no qual detalha o problema e as abordagens para melhorar o desempenho do programa. Depois disso, atualize os arquivos em specs/008-incremental-path-reservations para apontar para esse arquivo de documentação
```

| Item | Before | After |
| --- | --- | --- |
| Background reference | Context reached through the approved plan and its separate source documents. | Direct link to `docs/incremental_path_reservatons.md`, consolidating the problem and improvement approaches. |
| Documentation inventory | General local-repair solver documentation only. | Also includes the consolidated technical rationale and validation evidence. |
| Contracts and acceptance | Requirements of the approved specification. | Preserved; documentation consolidation adds no algorithmic or API changes. |
