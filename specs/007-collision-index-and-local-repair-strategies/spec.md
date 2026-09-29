Created: 2026-09-16T19:46:38-03:00

Author: Lucas Martello Nogueira (local user: lucas; repository Git identity)

Last updated: 2026-09-16T19:56:44-03:00

AI model: GPT-6 (Codex)

# Specification: Collision Indexes and Local Repair Strategies

## 1. Status, authority, and prompt

This is the implementation specification derived from the approved [plan](plan.md), including `adjustments / adjustment 1`. It specifies future code changes; creating this document does not implement the feature. Paths below are repository-relative. Existing function names are navigation anchors; proposed names are identified as additions.

### User request

```text
specs/007-collision-index-and-local-repair-strategies/plan.md aprovado, agora crie o arquivo spec, com mais detalhes de implementação no código e onde que essas implementações devem ser feitas
```

### Binding adjustments

1. Write the actual variable, parameter, return, and container types explicitly, even when long. Do not introduce type aliases with `using`, `typedef`, or similar shortcuts. Examples below also avoid inferred variable types. Concrete domain structs and enums remain allowed.
2. Implement all detector changes directly in `getCollision`. Remove `getCollisionV1`; do not retain a wrapper delegating to V1 or expose a second production detector.
3. Do not create or edit test files or test-support files, and do not register new test targets. Correctness criteria and execution of existing compatible checks remain part of validation. Document any legacy test incompatibilities caused by the API migration.

The approved plan and these adjustments supersede conflicting historical requirements in [003](../003-solution-collision-reporting/spec.md), [004](../004-local-path-repair-parallel-solver/spec.md), and [005](../005-cli-results-and-iterative-local-repair/spec.md): flat pairwise conflict vectors, solver-time duplicate-endpoint rejection, and unrestricted suffix processing during local repair are replaced. Keep the historical documents and their prompts intact. Preserve stay-at-target semantics, aligned path slots, sequential repair, full-path fallback, partial results, and existing artifact organization.

Use [.github/mapf.md](../../.github/mapf.md), [getCollisions.md](../../getCollisions.md), and [ref.md](../../ref.md) as domain context, interpreted through the approved adjustments. The working tree already contains a V1 draft and user changes to MAPF assumptions; incorporate their intent when implementing. The unrelated deletion of `tutorial_perf_flamegraph_mapf.md` is outside this specification.

## 2. Objective and implementation boundaries

Replace repeated pairwise detection with occupancy aggregation and one repair record per explicit participating path. Maintain global event membership separately so reporting and solution validity still include agents parked at their destinations. Add independently unique starts/goals, selectable repair order, and a bound at the active agent's next distinct conflict for local suffix processing.

Keep C++20, the standard library, CMake, and the existing A*/SIPP solvers. No dependency addition is required. The detector uses value keys, hash maps/sets, variants, and sorted vectors. SIPP safe intervals and reverse-edge reservations remain the search mechanism.

Global collision and reservation reconstruction may still read full paths after a tentative repair. The bound applies to local temporal suffix evaluation, not to full snapshot reconstruction, copying path containers, SIPP search expansion, or the separate full-path fallback. Do not implement incremental occupancy deletion or timestamp-preserving reconnection in this feature.

### File and symbol ownership

| File | Existing anchor or new declaration | Required implementation |
| --- | --- | --- |
| `include/mapf/core/instance.hpp` | `Instance` private section | Declare `bool isValidInstance() const`. |
| `src/mapf/core/instance.cpp` | Both constructors; anonymous namespace | Add coordinate keys/hash and endpoint validation; throw before construction completes. |
| `include/mapf/core/cell_conflict.hpp` | `CellConflict` | Replace reference/const fields with assignable pointer/value fields. |
| `include/mapf/core/edge_conflict.hpp` | `EdgeConflict` | Store assignable canonical endpoint pointers and arrival time. |
| New `include/mapf/core/conflict_key.hpp` | `CellTime`, `EdgeTime`, hashes, ordering functions | Declare reusable value keys and deterministic record ordering. |
| New `src/mapf/core/conflict_key.cpp` | Implementations of the preceding declarations | Implement equality, complete hashes, canonical edge construction, and variant comparison. |
| `include/mapf/core/solution_conflicts.hpp` | `SolutionConflicts`; new `VertexEvent`, `EdgeEvent` | Replace flat vectors with event maps and sorted per-agent records; add `empty()`. |
| `include/mapf/utils.hpp` | Existing global `getCollision`, `validateSolution` declarations | Keep their signatures and expose no V1 function. Document revised contracts. |
| `src/mapf/utils.cpp` | `getCollisionV1`, `getCollision`, `validateSolution` | Remove V1; replace the pairwise body with validated aggregation; migrate the boolean predicate. |
| `include/mapf/solvers/local_path_repair_solver.hpp` | `PathReservationState`, `LocalPathRepairResult`; new enum | Add `LocalRepairStrategy`; keep diagnostic snapshots; spell modified container types explicitly. |
| `include/mapf/solvers/local_path_repair_parallel_solver.hpp` | Constructor and private members | Add a defaulted strategy parameter and member. |
| `include/mapf/solvers/local_path_repair_iterative_solver.hpp` | Constructor and private members | Add the same strategy contract. |
| `src/mapf/solvers/local_path_repair_parallel_solver.cpp` | Constructor, `solve` | Validate/store/forward strategy; retain parallel initial A* only. |
| `src/mapf/solvers/local_path_repair_iterative_solver.cpp` | Constructor, `solve` | Validate/store/forward strategy; retain sequential initial A*. |
| `src/mapf/solvers/local_path_repair_solver_common.hpp` | `repairInitialPaths` | Append the strategy parameter. |
| `src/mapf/solvers/local_path_repair_solver_common.cpp` | Selection, candidate, suffix, synchronization, repair loop | Implement Sections 6–9 below; replace ownership scans and duplicate rebuilds. |
| `src/main.cpp` | `CliOptions`, `printUsage`, `parseOptions`, `runCli` | Parse/forward the strategy, populate metadata, remove caller-side normalization. |
| `src/mapf/experiments/experiment_utils.hpp` | `ExperimentRunResult`, `normalizeConflicts` | Add optional strategy metadata; remove the unused experiment conflict snapshot field. |
| `src/mapf/experiments/experiment_utils.cpp` | `normalizeConflicts`, `validSolverMetadata`, `writeExperimentArtifacts` | Project shared detector events, validate strategy metadata, append a stats column. |
| `src/mapf/experiments/manual_experiment.cpp`, `src/mapf/experiments/benchmark_experiment.cpp` | `ExperimentRunResult` aggregate initialization | Remove redundant conflict computation/initializers and preserve priority-planning metadata. |
| `src/mapf/solvers/priority_planning_solver.cpp` | `summarize` | Audit its completeness check plus `validateSolution`; change only if required by migration. |
| `CMakeLists.txt` | `add_library(mapf ...)`; optional new option/target | Register `conflict_key.cpp`; optionally add the benchmark outside the test block. |
| Optional new `tools/collision_benchmark.cpp` | Standalone executable | Independent private oracle, deterministic valid inputs, correctness comparison, and timing. |
| New `docs/collision_index_and_repair_strategies.md` | Feature guide | Document the representation, API, scheduling, repair bound, and measured costs. |
| `docs/local_path_repair_parallel_solver.md`, `docs/local_path_repair.md`, `docs/path_sufix_repair_algorithm.md` | Selection and suffix descriptions | Describe bounded processing and identify superseded whole-suffix guidance. |
| `docs/experiment_cli_and_results.md`, `readme.md` | CLI and artifact contracts | Add strategy examples/default and exact stats schema. |
| `.github/mapf.md` | Project-specific assumptions | Clarify independent uniqueness of starts and goals and pre-search rejection. |

`CMakePresets.json`, run/profiling scripts, SIPP public APIs, `Result`, historical results, and test files are outside the required change set. Use explicit standard-library types in new or rewritten code; do not start a repository-wide cleanup of unrelated existing aliases.

## 3. Domain and error contracts

### Identity, occupancy, and ownership

- `pathIndex` is the position in `paths` and `instance.getAgents()`. Store it as `int` in participant sets after checking representability. It is not `Agent::id` or `scenarioId`.
- Starts must be pairwise distinct; goals must independently be pairwise distinct. An agent may start at its own goal, and another agent may start at that goal. Scenario buckets need not be unique.
- All cell pointers belong to one live grid. Pointer identity is valid only within that grid. Caller-owned cells must outlive snapshots; moving/destroying a grid while snapshots are used is invalid.
- Time is absolute and begins at zero. A nonempty path ends at `path.size() - 1`. Empty paths occupy their original slots and generate no occupancy. A collision-free snapshot alone does not prove all required paths exist.
- A destination owner is explicit at its arrival time. It is virtual strictly afterward. Virtual owners participate in global vertex events but receive no repair record at those later times.
- A swap uses the transition arriving at `time`, with opposite directions over a nondegenerate edge. Waits and same-direction traversals do not independently emit edge events. Same-direction co-occupancy is represented by vertex events. Following and collision-free cycles remain allowed.
- No records need be generated after all paths finish: unique destinations prevent a collision involving only virtual owners.

### Error behavior

| Boundary | Required behavior |
| --- | --- |
| Either `Instance` constructor, repeated start or repeated goal | Throw `std::invalid_argument`; no valid instance is returned. |
| `getCollision`, repeated nonempty-path start/goal, null cell, unrepresentable index/time | Throw `std::invalid_argument` before relying on the invalid value. |
| `validateSolution`, one of those detector input errors | Return `false`; catch `std::invalid_argument` specifically. |
| Strategy constructor/common-engine API, invalid enum value | Throw `std::invalid_argument` before path search/repair. |
| CLI flag errors or invalid instance construction | Existing outer `runCli` catch returns 2; no result bundle. |
| Unrepairable solution or missing initial path | Preserve aligned partial diagnostics and return unsuccessful metrics; CLI returns 1 after writing the appropriate artifacts. |
| Artifact normalization/metadata failure | `writeExperimentArtifacts` returns `false` through the existing writer error contract. |

The detector can validate null pointers and full-path endpoints, but cannot prove the lifetime/grid ownership of arbitrary non-null pointers. Keep that as a documented caller precondition. Structural path validation in the repair engine additionally checks free cells, true endpoints, and legal moves.

## 4. Endpoint validation in `Instance`

In `include/mapf/core/instance.hpp`, place `bool isValidInstance() const;` beside the private state. The function inspects `agents` and performs no mutation.

In the anonymous namespace of `src/mapf/core/instance.cpp`, add a concrete `CoordinateKey` with `int x`, `int y`, equality, and `CoordinateKeyHash`. Hash both coordinates using `std::hash<int>` and unsigned combination; do not pack signed coordinates with unsafe shifts or concatenate strings. Include `<unordered_map>` and `<functional>`.

Implement the member in `namespace mapf`, before the accessors:

```cpp
bool Instance::isValidInstance() const {
    std::unordered_map<CoordinateKey, int, CoordinateKeyHash> startOwners;
    std::unordered_map<CoordinateKey, int, CoordinateKeyHash> goalOwners;
    startOwners.reserve(agents.size());
    goalOwners.reserve(agents.size());

    for (const Agent& agent : agents) {
        const CoordinateKey start{agent.startPosition.x, agent.startPosition.y};
        const CoordinateKey goal{agent.goalPosition.x, agent.goalPosition.y};
        if (!startOwners.emplace(start, agent.id).second ||
            !goalOwners.emplace(goal, agent.id).second) {
            return false;
        }
    }
    return true;
}
```

The two maps are independent. A duplicate in either map is sufficient to reject the instance. Store real IDs as the values for diagnostics; no contiguous-ID assumption is needed.

Insert the following check at the end of the manual constructor body after its existing ID/free-cell loop. Insert it in the file constructor after `agents = std::move(parsedAgents)` and `grid = Grid(&freeCells, height, width)`, before returning:

```cpp
if (!isValidInstance()) {
    throw std::invalid_argument(
        "Agent starts and agent goals must each be unique."
    );
}
```

Do not use the existing `require` helper for this check: it throws `std::runtime_error`. Keep all current map/scenario/ID validations and validate only the loaded scenario prefix requested by `numAgents`. An empty agent collection is valid.

## 5. Conflict model and the single detector

### 5.1 Assignable records and value keys

Replace the declarations in the two existing conflict headers with these fields, inside `namespace mapf`:

```cpp
struct CellConflict {
    Cell* cell;
    int time;
};

struct EdgeConflict {
    Cell* cell_1;
    Cell* cell_2;
    int time;
};
```

`cell_1` and `cell_2` are canonical endpoints, ordered lexicographically by `(position.x, position.y)`. They do not encode the active agent's travel direction. Pointer/value fields make records assignable and sortable; do not retain const data members or references.

Declare the following in new `include/mapf/core/conflict_key.hpp`. Use `#pragma once` and include `<cstddef>`, `<variant>`, and the two record headers:

```cpp
namespace mapf {
    struct CellTime {
        Cell* cell;
        int time;
        bool operator==(const CellTime& other) const;
    };

    struct EdgeTime {
        Cell* first;
        Cell* second;
        int time;
        bool operator==(const EdgeTime& other) const;
    };

    struct CellTimeHash {
        std::size_t operator()(const CellTime& key) const;
    };

    struct EdgeTimeHash {
        std::size_t operator()(const EdgeTime& key) const;
    };

    bool cellComesBefore(const Cell* first, const Cell* second);
    EdgeTime makeEdgeTime(Cell* from, Cell* to, int arrivalTime);
    int conflictTime(const std::variant<CellConflict, EdgeConflict>& conflict);
    bool conflictRecordComesBefore(
        const std::variant<CellConflict, EdgeConflict>& first,
        const std::variant<CellConflict, EdgeConflict>& second
    );
}
```

Implement these in `src/mapf/core/conflict_key.cpp`. Equality compares every field by value; hash every equal-compared field, using `std::hash<Cell*>` for cells and `std::hash<int>` for time. A file-private unsigned `combineHash` helper may be reused. For example:

```cpp
std::size_t combineHash(std::size_t seed, std::size_t value) {
    return seed ^ (value + static_cast<std::size_t>(0x9e3779b9U) +
                   (seed << 6) + (seed >> 2));
}

std::size_t mapf::EdgeTimeHash::operator()(const EdgeTime& key) const {
    std::size_t seed = std::hash<Cell*>{}(key.first);
    seed = combineHash(seed, std::hash<Cell*>{}(key.second));
    return combineHash(seed, std::hash<int>{}(key.time));
}
```

Hash iteration must never determine scheduling or export order. `makeEdgeTime` uses coordinate ordering, not relational pointer comparisons. Do not reuse SIPP's `StateKey::intervalIndex` as a timestep or its directed `EdgeKey` as a canonical collision event.

The record comparator orders `(time, kind, first.x, first.y, second.x, second.y)`, where vertex precedes edge and a vertex uses its own cell as both comparison endpoints. Implement variant access with `std::get_if<CellConflict>`/`std::get<EdgeConflict>` and explicitly typed locals. The common engine and detector must reuse this comparator.

### 5.2 Global membership and per-agent snapshots

In `include/mapf/core/solution_conflicts.hpp`, include the key header, `<unordered_map>`, `<unordered_set>`, `<variant>`, and `<vector>`. Replace `cellConflicts` and `edgeConflicts` with:

```cpp
namespace mapf {
    struct VertexEvent {
        std::unordered_set<int> participants;
    };

    struct EdgeEvent {
        std::unordered_set<int> forward;
        std::unordered_set<int> reverse;

        bool active() const {
            return !forward.empty() && !reverse.empty();
        }
    };

    struct SolutionConflicts {
        std::unordered_map<CellTime, VertexEvent, CellTimeHash> vertexEvents;
        std::unordered_map<EdgeTime, EdgeEvent, EdgeTimeHash> edgeEvents;
        std::vector<std::vector<std::variant<CellConflict, EdgeConflict>>> byAgent;

        bool empty() const {
            return vertexEvents.empty() && edgeEvents.empty();
        }
    };
}
```

`forward` means canonical first-to-second, `reverse` means second-to-first. Store path indexes in all three participant sets. Derive counts from set sizes. There is no independently decremented scalar count.

The returned maps contain active events only: at least two vertex participants or nonempty edge sets in both directions. `byAgent.size() == paths.size()` always, including collision-free and empty-slot inputs. The emptiness of the outer vector is not a collision predicate.

A record refers to an event through its value key, not an iterator/pointer into a map. Each explicit `(pathIndex, event key)` appears once. Snapshots own their containers and can be copied safely; cell pointers remain non-owning. Preserve immutable copies of `initialConflicts` when replacing `remainingConflicts`.

### 5.3 Replace `getCollision` in `src/mapf/utils.cpp`

Keep the existing global signature from `include/mapf/utils.hpp`:

```cpp
mapf::SolutionConflicts getCollision(
    const std::vector<std::list<mapf::Cell*>>& paths
);
```

Delete `getCollisionV1` completely. Replace the old pairwise body of `getCollision`; remove its obsolete TODO and its private `positionAt` helper if unused. Use the shared canonical ordering instead of another private edge-order implementation. Preserve unrelated CSV behavior.

Add the necessary standard-library includes directly (`<limits>`, `<stdexcept>`, `<unordered_map>`, `<unordered_set>`, `<variant>`, and `<utility>` as used). Place only concrete detector-support structs/helpers in the anonymous namespace:

```cpp
struct GoalOccupancy {
    int i;
    int arrivalTime;
};
```

#### Pass A: validate paths and prepopulate endpoints

1. Check the path-count bound before allocating snapshot containers. Ensure the externally incremented `int i` cannot overflow, including after an empty slot; reject a path count greater than `INT_MAX`. For every nonempty path, check `path.size() - 1 <= INT_MAX` before conversion. Increment cell time only if another cell remains, or use a checked `std::size_t` position converted to `int`, so an arrival at `INT_MAX` does not trigger a final overflowing increment.
2. Construct the result and resize `byAgent` to `paths.size()` after the path-count check.
3. Create `std::unordered_map<mapf::Cell*, int> startOwners`, `std::unordered_map<mapf::Cell*, GoalOccupancy> goalVertexLookup`, and `std::vector<int> arrivalTimes(paths.size(), -1)`. Reserve endpoint maps by path count.
4. For each nonempty path, verify every cell pointer is non-null before dereferencing/using coordinates. Count explicit cells and non-wait moves using checked `std::size_t` arithmetic for capacity estimates. This validation pass remains linear in explicit input size.
5. Insert `path.front()` into `startOwners` and `path.back()` into `goalVertexLookup`; a failed insertion into either map throws `std::invalid_argument`. Save the path index and arrival time. Empty paths insert nothing.
6. Increment an external `int i = 0` once per vector slot, including empty paths. Reset it before the occupancy pass. Never compact or renumber paths.

Endpoint checks use pointer identity under the one-grid precondition. Instance construction uses coordinate identity because that API receives positions. All endpoint metadata must exist before the occupancy pass begins, independent of path order.

#### Pass B: aggregate occupancy

Use these temporary containers, reserving checked estimates from Pass A:

```cpp
std::unordered_map<mapf::CellTime, mapf::VertexEvent, mapf::CellTimeHash> vertexLookup;
std::unordered_map<mapf::EdgeTime, mapf::EdgeEvent, mapf::EdgeTimeHash> edgeLookup;
```

For each explicit cell at time `t`, insert `i` into its vertex membership set. Then consult the destination map:

```cpp
std::unordered_set<int>& occupants =
    vertexLookup[mapf::CellTime{cell, t}].participants;
occupants.insert(i);

const std::unordered_map<mapf::Cell*, GoalOccupancy>::const_iterator goalVertexLookupItem =
    goalVertexLookup.find(cell);
if (goalVertexLookupItem != goalVertexLookup.cend() &&
    goalVertexLookupItem->second.i != i &&
    goalVertexLookupItem->second.arrivalTime <= t) {
    if (goalVertexLookupItem->second.arrivalTime < t) {
        occupants.insert(goalVertexLookupItem->second.i);
    }
}
```

At equality, the owner's explicit traversal supplies its membership, regardless of traversal order. The strict inner comparison prevents treating that arrival as virtual; sets prevent duplicate owners when multiple visitors arrive together.

For `t > 0 && previousCell != cell`, construct `mapf::makeEdgeTime(previousCell, cell, t)`. Insert `i` into `forward` if `previousCell == key.first`, otherwise into `reverse`. Do not emit an event yet. A wait only contributes vertex occupancy. Do not use a flag that skips vertex aggregation after a goal lookup hit.

#### Pass C: materialize active events and sort

- For each vertex bucket with at least two participants, retain the event. For each participant `i`, emit `mapf::CellConflict{key.cell, key.time}` in `byAgent[i]` only if `key.time <= arrivalTimes[i]`. The participant came from explicit occupancy or a virtual goal insertion, so this comparison identifies repair ownership without rescanning the path.
- For each edge bucket with both directions present, retain the event and emit `mapf::EdgeConflict{key.first, key.second, key.time}` once for every member of both sets. A path cannot traverse the same edge in both directions at one arrival time.
- Move active membership sets into the result maps after reading any needed fields. Discard singleton vertex buckets and one-direction edge buckets.
- Sort every `std::vector<std::variant<mapf::CellConflict, mapf::EdgeConflict>>` in `byAgent` using `mapf::conflictRecordComesBefore`.
- Return the snapshot. Do not allocate an `agents × makespan` padded table, enumerate pairs, or pad finished paths with virtual waits.

### 5.4 `validateSolution` and expected cardinalities

Replace its flat-vector check with:

```cpp
bool validateSolution(const std::vector<std::list<mapf::Cell*>>& paths) {
    try {
        return getCollision(paths).empty();
    } catch (const std::invalid_argument&) {
        return false;
    }
}
```

Do not catch allocation failures or unrelated implementation exceptions as ordinary collisions. This remains a collision predicate: empty path slots alone do not make it return false. Keep the separate completeness checks in both local repair and priority planning.

| Occurrence | Global events | Global participants | Repair records |
| --- | ---: | ---: | ---: |
| Two explicit agents at a cell/time | 1 | 2 | 2 |
| Three explicit agents at a cell/time | 1 | 3 | 3 |
| Four explicit agents at a cell/time | 1 | 4 | 4 |
| One visitor at a previously occupied goal | 1 | 2 | 1 |
| Two visitors and a virtual goal owner | 1 | 3 | 2 |
| Goal owner at its exact arrival plus another explicit occupant | 1 | 2 | 2 |
| Three forward and one reverse edge traversal | 1 edge event | 4 | 4 edge records |
| All traversals have the same direction | 0 edge events | — | 0 edge records |

Vertex events implied by the last two rows remain independent and must also be emitted when present.

## 6. Strategy API and common-engine entry

### 6.1 Shared enum and adapter constructors

Add this concrete enum in `include/mapf/solvers/local_path_repair_solver.hpp`, inside `namespace mapf`, before the result/reservation structs:

```cpp
enum class LocalRepairStrategy {
    RESOLVE_BY_AGENT,
    RESOLVE_BY_TIME
};
```

Keep `LocalPathRepairResult::initialPaths`, `paths`, `pathCosts`, `initialConflicts`, `remainingConflicts`, and `reservations`. Their snapshot semantics change through `SolutionConflicts`, not through parallel result representations.

When editing `PathReservationState`, spell its fields as `std::unordered_map<Cell*, std::unordered_set<int>> vertex_agents` and `std::unordered_map<Cell*, int> goal_reservations`. Update the changed `hasOtherAgent` parameter accordingly instead of using `VertexAgents`. Remove the now-unneeded `VertexAgents`/`GoalReservations` aliases after migrating their production uses. Keep `SafeIntervalTable` as the existing concrete struct; this feature does not redesign SIPP storage.

Append parameters in the existing solver headers, preserving positional compatibility for existing callers:

```cpp
LocalPathRepairParallelSolver(
    const Instance& instance,
    std::size_t numberOfThreads,
    bool continueIfFailed = false,
    LocalRepairStrategy localRepairStrategy = LocalRepairStrategy::RESOLVE_BY_AGENT
);

explicit LocalPathRepairIterativeSolver(
    const Instance& instance,
    bool continueIfFailed = false,
    LocalRepairStrategy localRepairStrategy = LocalRepairStrategy::RESOLVE_BY_AGENT
);
```

Both classes store `LocalRepairStrategy localRepairStrategy;`. Update constructor definitions/initializers in their `.cpp` files. Retain the parallel solver's positive thread-count validation. Validate enum values before initial A*, even for an empty instance.

Add a shared internal `void validateLocalRepairStrategy(LocalRepairStrategy strategy);` declaration to `src/mapf/solvers/local_path_repair_solver_common.hpp` and implementation in the matching source under `mapf::local_path_repair_detail`. Its `switch` accepts the two enumerators and throws `std::invalid_argument` for anything else. Call it in both constructors and at the common-engine entry.

Append the strategy after `startedAt` in the common declaration and definition:

```cpp
LocalPathRepairResult repairInitialPaths(
    const Instance& instance,
    std::vector<std::list<Cell*>> initialPaths,
    bool continueIfFailed,
    std::chrono::steady_clock::time_point startedAt,
    LocalRepairStrategy localRepairStrategy
);
```

Update the final calls in both `solve()` methods to forward the member. The parallel adapter must still collect futures in instance order. Neither adapter implements conflict selection; all strategy behavior belongs in the common source. Do not parallelize repair commits.

### 6.2 Initialization in `repairInitialPaths`

Before starting the repair loop:

1. Validate the enum and `initialPaths.size() == agents.size()`.
2. For each nonempty path, check structural validity once against that agent's true start/goal. Reuse `structurallyValid` for this initial whole-path check. Reject malformed nonempty input as `std::invalid_argument`; empty initial paths remain ordinary search failures.
3. Set `result.initialPaths`, copy into `result.paths`, build `result.initialConflicts = getCollision(result.initialPaths)`, and copy that snapshot into `result.remainingConflicts`. Do not run the detector twice on the same initial state.
4. Build initial committed reservations once with `buildReservationState`. Initialize costs/metrics separately.
5. Preserve zero-agent success and missing-initial-path failure. Diagnostics must be aligned in both cases. No missing path may silently disappear from the arrays.
6. Remove `hasSharedStarts`, `hasDuplicateGoals`, and their return branch. Uniqueness is enforced by construction and the standalone detector boundary.
7. Initialize the snapshot revision, ignored-record set, per-agent cursors, and full-configuration history for the entire repair invocation.

Use the existing `vertex_agents` reservation values as real `Agent::id` values; `hasOtherAgent(..., agent.id)` retains that meaning. Collision membership uses path indexes. Do not interchange these two identity domains.

## 7. Deterministic selection and continuation

All additions in this section belong to the anonymous namespace in `src/mapf/solvers/local_path_repair_solver_common.cpp`, unless stated otherwise.

### 7.1 Replace global ownership scans

Remove the implementation of `earliestExplicitConflict` that scans `cellConflicts`/`edgeConflicts` and reconstructs an active path. Replace it with selection from `conflicts.byAgent[pathIndex]`. The new record already establishes ownership.

Replace the old `ActiveConflict` carrier with an owning selection value, for example:

```cpp
struct SelectedConflict {
    std::size_t pathIndex;
    std::size_t revision;
    std::variant<CellConflict, EdgeConflict> conflict;
};
```

Keep no references/iterators from a previous snapshot in this struct. Add helpers with these responsibilities:

| Proposed helper | Inputs and result | Required behavior |
| --- | --- | --- |
| `unresolvedConflictFingerprint` (adapt existing) | Path index and `const std::variant<CellConflict, EdgeConflict>&`; returns `std::string` | Encode owner, kind, absolute time, and canonical coordinates with unambiguous delimiters. |
| `firstEligibleConflict` | One path index, snapshot, ignored set, cursor, revision; returns `std::optional<SelectedConflict>` | Advance over ignored entries in that agent's sorted vector; never infer ownership from path coordinates. |
| `selectByAgent` | Snapshot, ignored set, cursors, revision | Choose the lowest path index with an eligible record, then its first eligible record. |
| `selectByTime` | Same inputs | Compare the first eligible record of every agent; minimum by record comparator, then path index. |
| `selectionIsCurrent` | Selection, snapshot, current revision | Verify revision, active global event, membership, and existence of the selected per-agent record before repair. |

Use `std::unordered_set<std::string>` for ignored fingerprints and `std::vector<std::size_t>` for cursors. Clear/reset both when replacing a snapshot. Cursors only skip ineligible entries; they do not remove real events from the snapshot.

Add a real selection `switch` inside `repairInitialPaths` and replace the outer agent-major `for` with a shared selection/repair loop:

```cpp
std::optional<SelectedConflict> selected;
switch (localRepairStrategy) {
case LocalRepairStrategy::RESOLVE_BY_AGENT:
    selected = selectByAgent(
        result.remainingConflicts, ignoredForRevision, cursors, revision
    );
    break;
case LocalRepairStrategy::RESOLVE_BY_TIME:
    selected = selectByTime(
        result.remainingConflicts, ignoredForRevision, cursors, revision
    );
    break;
default:
    throw std::invalid_argument("Unknown local repair strategy.");
}
```

After every commit, select again from the new snapshot. Agent order revisits a lower index if new conflicts involve it. Time order compares across all agents, not only within the currently selected agent. With no eligible selection, finalize against actual remaining events; ignored conflicts still cause failure.

Head comparison is at most `O(A)` per selection, plus advancing cursors over ignored records. Start with this approach; a global priority heap is deferred.

### 7.2 Same-time conflicts and failed attempts

The selected time `tc` represents all of the active agent's events at that time. The local candidate must remove every such event involving that agent. A vertex event sorts before an edge event, but that tie rule must not allow the edge event at `tc` to survive an accepted candidate.

When local candidates and full fallback fail:

- `continueIfFailed == false`: finalize immediately, preserving committed paths and current diagnostics.
- `continueIfFailed == true`: add the selected `(pathIndex, event key)` fingerprint to the current revision's ignored set and select again. Other owners, other events at the same time, and later conflicts remain eligible. Repeated failures eventually exhaust the finite records of that unchanged revision.

Ignored records are scheduling exclusions only. They remain in global maps, per-agent lists, reporting, next-conflict boundary calculation, and final failure evaluation. A successful commit clears ignored records and cursors because their assumptions refer to the old state.

### 7.3 Protection against cycling

Replace the existing `repairFingerprint` history scoped inside one agent loop. Maintain full path-configuration history for the whole invocation, including path-slot boundaries, empty slots, and waits. Serialize coordinates and path lengths unambiguously, or use a hash followed by exact equality verification. A hash collision alone must not reject a state.

Also track attempted selections with their eligible/ignored state for the same path configuration. Do not include the monotonically increasing revision in the equality used to detect a return to an earlier configuration: doing so would conceal cycles. Revision is for snapshot validity, not semantic state identity.

Reject a candidate that returns to an already committed full configuration; try other bounded candidates and then full fallback. Never reset this history when changing active agents. If no candidate progresses, follow the same continuation/failure policy. This guarantees rejection of repeated configurations; do not claim a fixed bound on all SIPP searches or all possible distinct delayed paths.

## 8. Bound local suffix processing at the next conflict

### 8.1 Window and candidate metadata

Add concrete private structs beside the new selection carrier:

```cpp
struct RepairWindow {
    std::size_t prefixEnd;
    std::size_t reconnectIndex;
    std::size_t endOldInclusive;
    std::optional<int> nextConflictTime;
};

struct LocalCandidate {
    std::vector<Cell*> path;
    int validatedThrough;
    int suffixTimeShift;
};
```

All three indices in `RepairWindow` refer to the old path at the selected revision. `validatedThrough` is the absolute arrival time at the end of the locally validated segment in the new path. `suffixTimeShift` is that arrival time minus `endOldInclusive`; recompute it after any inserted wait or mini-path. Do not assume it equals the initial bridge's shift.

Add a private `makeRepairWindow` helper, returning `std::optional<RepairWindow>`, using the selection, old path length, and the selected agent's sorted conflict list:

1. `tc = conflictTime(selected.conflict)`.
2. Find the first distinct conflict time `tn > tc` in the same agent list. Include ignored records; exclude same-time records from the next boundary.
3. If `tn` exists, set `endOldInclusive = tn - 1`; otherwise use `oldPath.size() - 1`.
4. Set the initial prefix anchor to `tc - 1`. A time-zero conflict cannot occur for valid distinct starts; handle it defensively as a non-repairable/inconsistent state without unsigned underflow.
5. For a selected vertex, the initial `reconnectIndex` is `tc + 1`; for an edge, it is `tc`. Check integer bounds before addition/conversion.
6. Require `prefixEnd < reconnectIndex <= endOldInclusive < oldPath.size()`. If no such initial window exists, use the full-path fallback; do not read through the next conflict to obtain a reconnection.
7. Try the existing `adaptiveAnchors(prefixEnd)` sequence for backward expansion. Expanding the prefix does not expand `endOldInclusive`. A candidate whose changed/validated segment crosses an older ignored conflict must repair that conflict or be rejected.

Example: current conflict at old time 5 and next at 11 gives `endOldInclusive = 10`. If the repaired bounded segment reaches old index 10 at new time 13, the untouched tail starts with old index 11 at new time 14. The next selection must use the rebuilt snapshot's actual times, not the old value 11.

### 8.2 Change the existing candidate helper interfaces

Change `buildLocalCandidate` to return `std::optional<LocalCandidate>` and accept one window instead of separate unbounded prefix/suffix indices:

```cpp
std::optional<LocalCandidate> buildLocalCandidate(
    Grid& grid,
    const Agent& agent,
    const std::vector<Cell*>& oldPath,
    const RepairWindow& window,
    const std::list<Cell*>& bridge,
    const PathReservationState& committedState,
    const PathReservationState& repairState
);
```

Change `appendScenarioTwoPointThreeSuffix` to accept `endOldInclusive` after `suffixStartIndex`. Retain the existing SIPP and reservation inputs. Its old-path transition loop must be:

```cpp
for (std::size_t oldIndex = suffixStartIndex;
     oldIndex < endOldInclusive;
     ++oldIndex) {
    Cell* next = oldPath[oldIndex + 1];
    // Evaluate this transition using its new absolute arrival time.
}
```

Validate the bound before entering this loop. `nextIsGoal` is true only when `oldIndex + 1 == oldPath.size() - 1`, not merely because the next cell is the end of this window. An intermediate reconnection/boundary uses transient SIPP acceptance.

In `buildLocalCandidate`, replace the current potential-conflict loop ending at `oldPath.size()` with a loop restricted to `[window.reconnectIndex, window.endOldInclusive]`. Copy prefix cells through `prefixEnd`, append the bridge without duplicating its first cell, then apply the suffix scenario only within that bound.

### 8.3 Preserve the five suffix cases

| Existing case | Required bounded behavior |
| --- | --- |
| Equal reconnection arrival | Reuse old cells through `endOldInclusive` at the old timing. |
| Earlier arrival with potential interference in the bounded segment | Wait safely at reconnection until the old time, then process the bounded segment. Reject if the wait cannot fit in one safe interval. |
| Earlier arrival without such interference | Permit the earlier timing through the bounded segment; compute the final shift. |
| Later arrival with potential interference | Run `appendScenarioTwoPointThreeSuffix` only through `endOldInclusive`, retaining direct/wait/mini-SIPP choices and reverse-edge checks. |
| Later arrival without such interference | Reuse the bounded cells at delayed times, with the same local temporal validity contract. |

Potential interference is a spatial lookup, not proof of temporal safety. Validate changed moves/waits in the bounded segment using `intervalAt`, `edgeBlocked`, `earliestSafeArrival`, and `appendDirectTransition` as applicable. Every newly generated bridge and mini-path must have legal steps and absolute-time alignment. Keep `appendWithoutFirst` to avoid duplicating the splice cell.

Do not call `structurallyValid` on the entire candidate after every local anchor. Initial unchanged paths have already been checked. Check the new bridge/modified segment and the splice endpoints; copied cell sequences inherit their original structural validity. Full replacement fallback is still checked in full.

### 8.4 Attach the deferred tail

After validating the bounded segment, record its ending time and shift. Copy old cells beginning at `endOldInclusive + 1` to the candidate without running suffix SIPP, spatial potential-conflict scans, or temporal checks over that tail. Construct the resulting full list/vector for the shared detector; copying is permitted and must be reported separately from temporal evaluation.

Check structural adjacency at the splice. The first deferred cell's new arrival must be strictly greater than `tc`. If an early bridge would place it at/before `tc`, insert a safe wait within the validated segment or reject the candidate. Use checked arithmetic for all new times; reject candidate times that exceed the finite range supported by the existing SIPP interval sentinel, without changing the standalone detector's integer contract.

The first edge into the tail is evaluated by the global candidate snapshot. A collision on that edge strictly after `tc` may remain as the next repair event. It must not force an unbounded local suffix traversal.

Retain a single permanent-goal interval query for the candidate's actual final cell and new arrival, even when it is in the copied tail: a complete candidate cannot reserve an unsafe final goal indefinitely. The existing `intervalAt` helper scans the goal cell's intervals; this is not necessarily constant-time, but it does not traverse the deferred path tail. When there is no next conflict, the bounded segment naturally reaches the real goal and must validate permanent occupancy there as part of local processing.

## 9. Transactional snapshots, fallback, and finalization

### 9.1 Candidate evaluation

Remove `earliestConflictWithFrozenPaths` and its pairwise `positionAt` helper once no call remains. Both local progress and full fallback verification must use the same `getCollision` contract.

For each selected attempt, build one reservation state excluding the active agent and reuse it across all anchors. The committed `result.reservations` remains unchanged while building candidates. For each candidate that passes bounded local validation:

1. Build a provisional aligned `std::vector<std::list<Cell*>> candidatePaths`, replacing only the active slot.
2. Compute `SolutionConflicts candidateConflicts = getCollision(candidatePaths)` exactly once for that candidate. This global pass accounts for shifted tails, new arrival times, newly created events, and obsolete events.
3. Check explicit active-agent progress from `candidateConflicts.byAgent[activeIndex]`. Also inspect global membership for virtual participation by the active agent; an empty repair list is insufficient.
4. Reject if any active-agent conflict remains at `tc`. Reject any new active-agent event at/before `tc`. For continuation, an older ignored event may remain only if its key, active-agent occupancy role, and participant membership are unchanged and its time lies outside the changed/validated segment. For edges, compare both directional sets, not just a total count. If an expanded anchor crosses that old event, it must disappear.
5. Later conflicts in the deferred tail may remain. Do not reject merely because the entire candidate snapshot is nonempty: other agents can still have unresolved conflicts.
6. Apply repeated-state checks. On failure discard provisional data and try the next anchor; never decrement committed membership or mutate a live reservation table.

The per-agent vector provides normal explicit progress information. The global-membership check is necessary for a path that arrives earlier and becomes a virtual owner at a conflicting time. Prefer event-map lookup/filtering over re-enumerating path pairs.

### 9.2 Commit sequence

After candidate conflicts and cycle checks succeed, construct the provisional committed reservations once. Calculate any new costs needed before publishing. Use owning temporary objects, for example a private struct holding `candidatePaths`, `candidateConflicts`, and `candidateReservations`.

Publish all three as one logical state transition. The repair phase is sequential, so this means no selection/finalization observes a mixture of old paths and new indexes. Prepare allocating operations before the transition and use moves/swaps of the complete state to preserve exception safety.

Increment the revision; clear ignored records and per-agent cursors; discard old windows/selections; then select again. Keep `result.initialConflicts` unchanged. Do not immediately recompute `getCollision` or reservations for the just-published paths.

### 9.3 Full-path fallback

If no bounded candidate works, retain the existing fallback based on `pathsExcept` and `AStarSippSolver::solve(grid, start, goal, otherPaths)`. This overload uses the permanent-goal policy. It may build its own safe intervals; account for that separately rather than claiming it reuses the local table.

Validate the complete fallback path structurally, then build its provisional snapshot once. Acceptance requires no event involving the active path anywhere, including virtual goal participation; unrelated agents may still conflict. Apply the same cycle protection and transactional publication. A failed fallback preserves the entire committed state and applies Section 7.2's continuation policy.

Do not change the SIPP overload API merely to merge fallback and local table construction. Such an optimization is outside the required change set.

### 9.4 Refactor `synchronizeResult`

Separate snapshot construction from metrics/finalization. The current helper always calls both `getCollision` and `buildReservationState`; remove those unconditional calls from the path used after each commit.

Keep a private metrics helper, such as `updateResultMetrics`, that reads committed paths/snapshots, fills `pathCosts`, sum of costs, makespan, injustice, and elapsed time, and never rebuilds indexes. Success requires requested success, all aligned required paths present, and `result.remainingConflicts.empty()`.

A final authoritative detector check is permitted once when returning; reuse current reservations unless path state changed. If the final check disagrees with the cached state, do not report success based on the cache. Initial and final diagnostics must retain the new per-agent/global representation on all return paths.

`src/mapf/solvers/priority_planning_solver.cpp::summarize` already checks required path existence before `validateSolution`. Preserve this behavior; no separate priority-planning strategy is added.

## 10. CLI integration in `src/main.cpp`

### 10.1 Option storage and parsing

Append these members to the anonymous-namespace `CliOptions`:

```cpp
mapf::LocalRepairStrategy localRepairStrategy =
    mapf::LocalRepairStrategy::RESOLVE_BY_AGENT;
bool localRepairStrategyProvided = false;
```

Add an anonymous-namespace parser beside `parseBoolean`:

```cpp
mapf::LocalRepairStrategy parseLocalRepairStrategy(const std::string& value) {
    if (value == "RESOLVE_BY_AGENT") {
        return mapf::LocalRepairStrategy::RESOLVE_BY_AGENT;
    }
    if (value == "RESOLVE_BY_TIME") {
        return mapf::LocalRepairStrategy::RESOLVE_BY_TIME;
    }
    throw std::invalid_argument(
        "-localRepairStrategy accepts only RESOLVE_BY_AGENT or RESOLVE_BY_TIME."
    );
}
```

In `parseOptions`, add `std::optional<std::string> localRepairStrategy;` alongside the other raw option values. Add `else if (flag == "-localRepairStrategy") { target = &localRepairStrategy; }` to the existing target-selection chain. Reuse its existing duplicate and missing-value checks. After raw parsing, convert the optional string and mark `localRepairStrategyProvided` when supplied.

In the `PriorityPlanningSolver` compatibility branch, reject any explicitly supplied strategy, including the default value. Accept both values for both local solvers. Preserve current thread/continuation restrictions. Names are case-sensitive, use one leading hyphen, and have no shorthand aliases.

Extend `printUsage` to include `[-localRepairStrategy <RESOLVE_BY_AGENT|RESOLVE_BY_TIME>]`, identify that it applies to local solvers, and document `RESOLVE_BY_AGENT` as the default. Keep help text consistent with the actual parser.

### 10.2 Dispatch and metadata

In `runCli`, pass `options.localRepairStrategy` as the appended constructor argument for both local solvers. Populate `run.localRepairStrategy` with that effective value in both local branches; leave it `std::nullopt` for priority planning. Preserve all current `numAgents`, thread, solver-name, and continuation fields.

Delete the assignment to `run.remainingConflicts` and its call to `normalizeConflicts` immediately before `writeExperimentArtifacts`. The writer is the single normalization owner for the final artifact bundle. Keep the existing inner artifact-error handling and outer CLI/instance-error handling.

The following invocations differ only in scheduling; other arguments follow existing CLI rules:

```bash
build/normal/mapf_app \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver -agents 100 -threads 8 \
  -localRepairStrategy RESOLVE_BY_TIME

build/normal/mapf_app \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairIterativeSolver -agents 100 \
  -localRepairStrategy RESOLVE_BY_AGENT
```

These are usage examples, not successful runs recorded by this document. The selected scenario prefix must satisfy the new endpoint constraints. The existing scripts already forward application arguments and need no flag-specific changes.

## 11. Experiment normalization and output

### 11.1 `ExperimentRunResult` in `experiment_utils.hpp`

Add `<optional>` and `mapf/solvers/local_path_repair_solver.hpp`. Append the field after the existing metadata fields to minimize disruption to aggregate initializer ordering:

```cpp
std::optional<LocalRepairStrategy> localRepairStrategy = std::nullopt;
```

Remove `std::vector<ConflictRecord> remainingConflicts` from `ExperimentRunResult`. Inspection of the current writer shows that it recomputes `finalConflicts` from `solutionPaths` instead of using this field. Remove its initializers/assignments from `src/main.cpp`, `manual_experiment.cpp`, and `benchmark_experiment.cpp` in the same migration step.

Keep `ConflictRecord`, `normalizeConflicts`'s public signature, and the artifact writer signatures. Do not remove `LocalPathRepairResult::initialConflicts` or `remainingConflicts`; those solver diagnostics are required and are a different API.

### 11.2 Replace pairwise normalization

Rewrite `src/mapf/experiments/experiment_utils.cpp::normalizeConflicts`:

1. Preserve its `paths.size() == instance.getAgents().size()` check.
2. Call `getCollision(paths)` once.
3. Produce one `ConflictRecord` per active vertex event and one per active edge event.
4. Map every participant path index to `agents[pathIndex].id`, including virtual destination owners. Sort and deduplicate IDs in each record.
5. Sort records by `(timestep, type, cell1.x, cell1.y, cell2.x, cell2.y)`, with `Vertex` preceding `Edge`. This preserves deterministic current export ordering despite unordered internal maps.

The following fragment illustrates the vertex projection, inside `namespace mapf::experiments`:

```cpp
const std::vector<Agent>& agents = instance.getAgents();
const SolutionConflicts snapshot = getCollision(paths);
std::vector<ConflictRecord> records;
records.reserve(snapshot.vertexEvents.size() + snapshot.edgeEvents.size());

for (const std::pair<const CellTime, VertexEvent>& entry : snapshot.vertexEvents) {
    std::vector<int> agentIds;
    agentIds.reserve(entry.second.participants.size());
    for (int pathIndex : entry.second.participants) {
        agentIds.push_back(agents[static_cast<std::size_t>(pathIndex)].id);
    }
    std::sort(agentIds.begin(), agentIds.end());
    agentIds.erase(std::unique(agentIds.begin(), agentIds.end()), agentIds.end());
    records.push_back(ConflictRecord{
        .cell1 = entry.first.cell,
        .cell2 = entry.first.cell,
        .timestep = entry.first.time,
        .type = ConflictType::Vertex,
        .agentIds = std::move(agentIds)
    });
}
```

For an edge, append IDs from both directional sets and use `key.first`, `key.second`, `key.time`, and `ConflictType::Edge`. Do not expand all cross-direction pairs and do not export one row per `byAgent` entry. Vertex records keep `cell1 == cell2`, matching existing CSV behavior.

Delete the old pairwise loops, their indexed-path/makespan materialization, `ConflictAccumulator`, and the old private `positionAt`. Remove `ConflictKey` and the private `cellComesBefore` if no remaining consumer needs them; the final vector comparator and shared canonical keys now supply their ordering role. Drop now-unused `<map>`/`<set>` includes after checking remaining uses.

### 11.3 Metadata normalization and stats schema

Extend `validSolverMetadata` to enforce:

| Solver | Optional strategy input | Effective output |
| --- | --- | --- |
| `PriorityPlanningSolver` | Must be absent; an explicit enum is contradictory metadata. | `-` |
| `LocalPathRepairParallelSolver` | Missing means default; if present, must be a valid enumerator. | `RESOLVE_BY_AGENT` or `RESOLVE_BY_TIME` |
| `LocalPathRepairIterativeSolver` | Same rule as parallel. | `RESOLVE_BY_AGENT` or `RESOLVE_BY_TIME` |

Keep existing thread/localRepair/continuation consistency checks. Use a private enum-to-string switch in `experiment_utils.cpp`; unknown enum values must fail metadata validation rather than silently becoming a default. Normalize a missing local value with `value_or(LocalRepairStrategy::RESOLVE_BY_AGENT)` without mutating the caller's const `run` object.

In `writeExperimentArtifacts`, append `local_repair_strategy` to `statsHeaders` and the corresponding normalized string to `statsRow`. Spell any rewritten CSV container declarations as `std::vector<std::string>` or `std::vector<std::vector<std::string>>`; introduce no new CSV alias.

The exact `_stats.csv` header is:

```text
map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,continue_if_failed,local_repair_strategy
```

The grouped `_conflicts.csv` header remains:

```text
cell_1,cell_2,timestep,conflict_type,agents
```

Preserve the `_solution.csv` schema, `agent_scenario_bucket`, and solver fields' existing placement in stats. The separate legacy `writeExperimentResult` has no strategy argument and is not the bundle writer; do not invent strategy metadata for it or change its existing legacy CSV contract.

### 11.4 Writer ownership and partial results

The writer continues to call `normalizeConflicts` once on `run.solutionPaths`. Use those records for both `conflictedAgentIds` and conflict CSV rows. `paths_resolved` and per-agent solution success must treat a virtual owner as conflicted even though its `byAgent` list is empty.

Keep the existing directory format `results/{git_branch}_{timestamp}/`, matching prefixes for stats/solution/conflicts, temporary-directory publication, and error cleanup. Keep the conditional conflicts file rule: failed local runs write it, including a header-only file if failure came from a missing path without geometric collisions. Do not rewrite historical output files.

In both example executables, remove `.remainingConflicts = normalizeConflicts(...)` from `ExperimentRunResult` initialization. They currently use priority planning, so absent strategy metadata remains correct. Preserve declaration order for any C++20 designated initializers.

## 12. Build registration and optional benchmark

Add `src/mapf/core/conflict_key.cpp` to `add_library(mapf ...)` near the other core sources in `CMakeLists.txt`. The existing public include directory already exposes the new header. Do not add test sources, alter test expectations, remove existing tests, or turn tests off in the normal preset to conceal incompatibilities.

If implementing the optional benchmark, use an opt-in executable outside `if(BUILD_TESTING)`:

```cmake
option(MAPF_BUILD_COLLISION_BENCHMARK "Build the collision detector benchmark" OFF)
if(MAPF_BUILD_COLLISION_BENCHMARK)
    add_executable(collision_benchmark tools/collision_benchmark.cpp)
    target_link_libraries(collision_benchmark PRIVATE mapf)
endif()
```

The option is additional CMake metadata, not an application runtime dependency. Keep its source and oracle self-contained. The oracle has an independently implemented pairwise traversal and is private to this executable; do not name it `getCollisionV1`, expose it from `mapf`, or share it through a test header.

Benchmark requirements:

- Generate the same valid full-path inputs once for both implementations; include distinct endpoints, empty slots, waits, revisits, asymmetric lengths, dense multi-agent meetings, parked owners, and opposite edge directions.
- Use deterministic seeds. Preserve input grid lifetime for both snapshots.
- Normalize oracle observations to canonical events with participant sets and direction sets, then derive explicit repair-owner lists. Legacy pair counts are not the comparison target.
- Compare canonical correctness before timing. Stop with a nonzero exit code on mismatch and report the seed/input case.
- Exclude generation, normalization for comparison, and I/O from detector timing. Consume results to prevent optimization from removing the work.
- Include warmup and repeated measurements; report median/spread, compiler/build flags, hardware, seed, path count, explicit cells, makespan, event/record counts, and memory measurements when available.

## 13. Documentation deliverables

Create `docs/collision_index_and_repair_strategies.md` during implementation with concrete examples of explicit versus virtual owners, edge direction membership, API migration, default/tie ordering, bounded old indices versus shifted new times, and snapshot publication. Include the benchmark command/procedure if the optional target is added, and record measured tradeoffs rather than promising a fixed speedup.

Update existing documents at the following conceptual anchors:

| Document | Required content |
| --- | --- |
| `docs/local_path_repair_parallel_solver.md` | Constructor examples, reservation identities, both selection strategies, revision-scoped continuation, bounded suffix cases, and separate fallback behavior. |
| `docs/local_path_repair.md` | Explain that reasoning about a reused suffix applies through the next explicit conflict boundary and that the deferred tail is reindexed globally. |
| `docs/path_sufix_repair_algorithm.md` | Bound Scenario 2.3's old-path transition loop and distinguish an intermediate boundary from the true goal. |
| `docs/experiment_cli_and_results.md` | Exact new flag/default, invalid uses, normalized strategy values, complete stats header, and grouped participant semantics. |
| `readme.md` | Entry-point examples for both strategies, uniqueness restriction, and link to the feature guide. |
| `.github/mapf.md` | State that repeated starts or repeated goals independently invalidate an instance before search; retain the user's conflict and stay-at-target constraints. |

Mark historical whole-suffix examples as superseded where they remain useful background. Do not edit the historical specification series to erase the old contract.

## 14. Ordered implementation steps

1. Implement `Instance::isValidInstance` and both constructor checks. Retain manual/file loading behavior other than endpoint uniqueness.
2. Introduce key/hash/ordering definitions, pointer/value records, and the new `SolutionConflicts`; register the core source in CMake.
3. Replace `getCollision` directly, delete V1, and update `validateSolution`. Migrate all production flat-vector consumers in the same working change so the application can build with the new representation.
4. Rewrite experiment projection and remove the unused experiment result field/caller normalizations. Preserve solver diagnostic snapshots.
5. Add the strategy enum, constructor members/defaults/validation, and common-engine parameter forwarding.
6. Replace common-engine ownership scans with deterministic per-agent selection, revision handling, ignored records, and invocation-wide cycle protection.
7. Introduce explicit repair windows and candidate metadata; bound local suffix loops and implement deferred-tail handling with checked new timestamps.
8. Replace pairwise candidate checks with provisional snapshots; implement progress checks, transaction publication, fallback verification, and metrics-only synchronization.
9. Wire the CLI flag, effective experiment metadata, and exact stats column. Update docs and example initializers.
10. Build production targets, execute compatible existing checks, validate the specified cases, and profile identical workloads. Implement the optional oracle benchmark if used to establish detector equivalence/speed. Record any tests blocked by the excluded migration.

Steps 2–4 are one representation migration: do not leave temporary compatibility vectors populated in production, a second detector, or an exporter using the old pairwise loops. This document authorizes no implementation work by itself; it specifies what to do when implementation is requested.

## 15. Validation and acceptance

Validation does not add/edit test files. Use code review, existing compatible executables, CLI runs, profiling, and the optional benchmark. When an existing check assumes removed fields or permits duplicate endpoints, report that incompatibility explicitly; do not claim an unchanged complete suite passed.

### 15.1 Required correctness evidence

| Area | Required scenarios and expected outcome |
| --- | --- |
| Constructor boundary | Duplicate start only, duplicate goal only, or both: `std::invalid_argument` in manual and file construction. Selected scenario prefix only is considered. |
| Valid endpoints | Zero agents, own start equal to own goal, another agent's start equal to a goal, sparse real IDs, and repeated scenario buckets remain supported. |
| Standalone detector | Duplicate endpoints/null cells/out-of-range conversions are rejected; `validateSolution` returns false for those errors. Empty slots preserve subsequent indexes. |
| Aggregation | Single path has no self-conflict; goal owner processed first or last gives equivalent remapped events; arrival equality and later virtual occupancy have the cardinalities in Section 5.4. |
| Multiplicity | Two/three/four explicit agents produce two/three/four repair records; multiple visitors deduplicate the parked owner. |
| Edge direction | Opposite moves create a swap; waits never do; removal of the only reverse participant removes the edge event without hiding independent vertex events. |
| Ordering | Mixed vertex/edge events and path-major discovery sort by the documented comparator; same-time events all participate in the progress check. |
| Strategy | Disjoint events at different times/indexes prove the two selection orders; agent ordering revisits earlier indexes after commits; time ordering selects the global minimum. |
| Adapter parity | Same initial paths and strategy give identical paths/metrics for parallel and iterative adapters except elapsed time; thread count does not alter ownership. |
| Boundaries | Multiple separated conflicts, consecutive conflicts, conflict at the goal, and absent next conflict produce the specified window/fallback behavior. |
| Suffix behavior | Equal, early, and late arrivals; safe/unsafe wait; Scenario 2.3; new tail collision; disappearing old event; no local temporal loop beyond `endOldInclusive`. |
| Snapshot consistency | Changed goal arrival, virtual participation, edge reservations, and all event memberships match a fresh full detector snapshot after commit. Initial diagnostics remain unchanged. |
| Failure/continuation | Failed local/fallback candidates leave committed state intact; later conflicts remain eligible after ignoring an earlier failure; ignored events remain visible; repeated configurations are rejected. |
| CLI | Omitted equals explicit agent strategy; both local solvers accept both values; missing/unknown/duplicate/incompatible flags return 2. Invalid instances produce no result bundle. |
| Artifacts | Exact stats header and effective strategy; one row per global event; sorted real IDs include virtual owners; final partial output and exit codes retain their contracts. |

### 15.2 Commands for the implementation phase

The following commands are planned verification, not results of creating this specification:

```bash
cmake --preset normal
cmake --build --preset normal --target mapf_app manual_experiment benchmark_experiment --parallel
cmake --build --preset normal --parallel
ctest --preset normal
cmake --preset profile
cmake --build --preset profile --parallel
```

Build production targets explicitly before the aggregate normal build because excluded legacy tests may no longer compile. If a test target failed to rebuild, an existing old binary is not valid evidence for the changed code: run only successfully rebuilt compatible targets and document the blocked ones. Leave the normal preset's test configuration intact.

If the optional benchmark is implemented:

```bash
cmake --preset profile -DMAPF_BUILD_COLLISION_BENCHMARK=ON
cmake --build --preset profile --target collision_benchmark --parallel
build/profile/collision_benchmark
```

Run representative solver experiments using the existing [profiling workflow](../../docs/perf_flamegraph_profiling.md). Compare strategies on identical valid scenario prefixes and thread counts; strategy-induced path/success changes must not be mistaken for detector speed differences.

### 15.3 Complexity and measurement contract

Let `A` be path slots, `S` explicit path cells, `H` makespan, and `C_i` the number of repair records for path `i`, with `C = sum(C_i)`.

- Old pairwise detector: `O(A²H)`.
- New detector: expected `O(A + S + C)` for validation/aggregation/materialization, plus `O(sum(C_i log C_i))` for sorting.
- Detector storage: `O(A + S + C)` without makespan padding. Hash-table bounds are expected, not worst-case guarantees.
- Global head selection: up to `O(A)` per selection, plus skipped ignored entries.
- Global snapshot/reservation rebuilds and complete path-container copying remain separate costs. Reservation rebuilding also traverses grid cells and merges occupied intervals; do not describe it as only conflict-count work.

Measure local suffix transitions, detector explicit-cell visits, snapshot rebuild time, reservation rebuild time/count, and total solve time separately. A valid final path does not prove the bound was respected; inspect loop bounds and profiling/private measurement evidence. Do not add public runtime logging solely for this validation.

Acceptance requires correct event equivalence and a demonstrated detector improvement on representative larger inputs. Do not promise a numerical speedup or memory reduction before measurement. If reservation rebuilding dominates total solve time, record that limitation.

### 15.4 Completion checklist

- Exactly one production collision function, `getCollision`, implements the new contract; no `getCollisionV1` declaration/definition/call remains in `include/` or `src/`.
- New/rewritten declarations use explicit types and introduce no aliases; all production consumers use the new event maps and per-agent records.
- Both `Instance` constructors enforce independent endpoint uniqueness; solver-level duplicate checks are removed.
- Both strategies are available through the constructor APIs and CLI, default to agent order, and follow deterministic tie and revision rules.
- Local suffix processing obeys the next-conflict boundary; changed-time tails are covered by the shared global snapshot; full fallback remains separate.
- Commits preserve path/index/reservation consistency, continuation preserves unresolved events, and cycle history survives agent changes.
- Final grouped exports, per-agent success, strategy metadata, and stats schema match this specification.
- Production builds and correctness/performance evidence are recorded with actual limitations. No test file/test-support file is added or edited and no new test target is registered.

## 16. Deferred work

- TODO: After profiling, evaluate compact participant storage for predominantly two-agent events without losing membership or directional information.
- TODO: Add a revision-aware global priority heap only if head selection is measurable in end-to-end profiles.
- TODO: If global rebuilds dominate, design incremental occupancy/reservation updates with per-agent provenance, changed goal-arrival handling, affected-neighbor discovery, and independent equivalence validation.
- TODO: Evaluate persistent/indexed path storage if copying the deferred tail remains costly; the present local bound does not eliminate copying.
- TODO: Study fixed-time reconnection separately if a future requirement forbids global tail reads; that changes which repairs can succeed.

## adjustments

The approved plan's `adjustment 1` is incorporated throughout this initial specification. No separate correction to this specification has been requested. Future corrections must record a timestamp, the complete request, and before/after changes, and update the audit timestamp.
