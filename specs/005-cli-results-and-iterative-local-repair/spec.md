* Created at: 2026-09-08 10:29:16 -03
* Author: lucas
* Last updated at: 2026-09-08 15:11:02 -03
* AI model used: GPT-5 (Codex)

# Spec: CLI Result Artifacts and Iterative Local Repair

## Source And Precedence

This specification implements the approved plan at
`specs/005-cli-results-and-iterative-local-repair/plan.md`.

All adjustments recorded in that plan are incorporated here. If the plan is
adjusted later, its newest adjustment overrides conflicting requirements in
this specification until the specification is updated.

This feature extends the behavior delivered by
`specs/002-mapf-priority-planning-experiments/spec.md` and
`specs/004-local-path-repair-parallel-solver/spec.md`. Requirements from those
specifications remain in force unless this specification explicitly replaces
them. In particular, the local-repair algorithm, reservation semantics,
adaptive repair window, suffix repair, and full-replanning fallback defined by
specification 004 must be preserved when their implementation is extracted
into shared code.

## Objective

Replace the hard-coded `mapf_app` demonstration with a validated experiment
CLI that loads a Moving AI map and scenario, runs one of three supported MAPF
solvers, and persists a consistent result bundle.

Every experiment that reaches a solver result must create
`results/{git_branch}_{timestamp}/` and write inside it:

- one statistics CSV; and
- one per-agent solution CSV.

A failed local-repair experiment must additionally write a final-conflicts CSV
in the same directory. The directory and all files belonging to one run must
use the same sanitized Git-branch and timestamp prefix.

Add `LocalPathRepairIterativeSolver`, which calculates unconstrained initial
paths sequentially and then uses exactly the same repair engine as
`LocalPathRepairParallelSolver`. Preserve the unconstrained path of every
agent, retain all available partial output on algorithmic failure, and derive
per-agent success and aggregate failure metrics from the final stored paths and
conflicts.

## Scope

This feature includes:

- strict parsing and validation of the `mapf_app` command line;
- explicit dispatch for `PriorityPlanningSolver`,
  `LocalPathRepairParallelSolver`, and `LocalPathRepairIterativeSolver`;
- a sequential initial-path adapter for local repair;
- extraction of the existing local-repair algorithm into a shared internal
  engine;
- optional continuation after an unrepairable conflict;
- storage of aligned unconstrained and final paths for every solver;
- preservation of the scenario file's first-column identifier;
- normalized final conflict records containing participating agent IDs;
- transactional stats, solution, and conditional conflict artifacts;
- per-experiment serialization of the agent count requested through `-agents`;
- per-experiment statistics serialization of the selected solver and effective
  conflict-repair continuation flag;
- meaningful partial metrics and per-agent status on solver failure;
- automated tests for the CLI, solvers, model changes, and artifacts; and
- user and developer documentation.

No third-party runtime dependency is required. Use the existing C++20 standard
library and the project's existing `Threads::Threads` dependency.

## Terminology And Core Invariants

- **Experiment agent ID** is `Agent::id`. It is unique within an instance and is
  the identifier used by solvers and conflict artifacts.
- **Scenario bucket** is stored internally as `Agent::scenarioId`. It preserves
  the first column of the Moving AI scenario row, is not required to be unique,
  and is serialized as `agent_scenario_bucket`.
- **Initial path** is the unconstrained A* path from an agent's start to its
  goal, ignoring all other agents.
- **Solution path** is the final path committed by the selected solver. It may
  still be conflicted when the solver reports failure.
- **Missing path** is an empty path and has cost `-1` in solution artifacts.
- **Path cost** is the number of actions: `path.size() - 1` for a non-empty
  path. A one-cell path is valid and has cost `0`.
- **Resolved path** is a non-empty final path whose agent participates in no
  normalized final conflict.
- **Algorithmic failure** means a solver returned a result with
  `success == false`. Available stats, paths, and conflicts must still be
  written.
- **Pre-result failure** means CLI parsing, instance construction, or solver
  execution threw before a solver result was obtained. No artifact bundle is
  written.

For every solver result, path vectors must be in `Instance::getAgents()` order:

```text
agents[i] <-> initialPaths[i] <-> solutionPaths[i]
```

Every result path vector must have exactly the same size as the agent vector,
including failure cases. Empty entries represent unavailable paths; vectors
must never be shortened or reordered to omit them.

## Public Model And Solver Interfaces

### Scenario identity

Extend `Agent` without changing the meaning of its existing ID:

```cpp
struct Agent {
    int id;
    int scenarioId = -1;
    Position currentPosition;
    Position startPosition;
    Position goalPosition;
};
```

The benchmark `Instance` constructor must assign the parsed first scenario
column to `scenarioId` while continuing to assign sequential, unique `id`
values starting at zero. Repeated first-column values must be retained exactly.

Manually created agents use `scenarioId == -1` unless their caller supplies a
different value. The default initializer must keep existing designated
initializers source compatible.

### Shared local-repair types

Create `include/mapf/solvers/local_path_repair_solver.hpp` and move these shared
declarations into it:

- `VertexAgents`;
- `GoalReservations`;
- `PathReservationState`; and
- `LocalPathRepairResult`.

`LocalPathRepairResult` must have this shape:

```cpp
struct LocalPathRepairResult {
    Result metrics;
    std::vector<std::list<Cell*>> initialPaths;
    std::vector<std::list<Cell*>> paths;
    std::vector<int> pathCosts;
    SolutionConflicts initialConflicts;
    SolutionConflicts remainingConflicts;
    PathReservationState reservations;
};
```

Required semantics:

- `initialPaths` is captured before any repair and never modified afterward;
- `paths` is initialized from `initialPaths` and contains the final committed
  state, including partial or conflicted state on failure;
- `pathCosts[i]` is `paths[i].size() - 1` for a non-empty path; solution CSV
  serialization independently uses `-1` for an empty entry;
- `initialConflicts` describes the complete initial-path vector;
- `remainingConflicts` is recomputed from the final returned `paths`;
- `reservations` describes the final returned `paths`; and
- `metrics.success` is true only when every final path exists and no final
  conflict remains.

`local_path_repair_parallel_solver.hpp` must include the neutral header so
existing callers that include only the parallel header remain source
compatible.

### Parallel local-repair solver

Extend the constructor without breaking existing call sites:

```cpp
LocalPathRepairParallelSolver(
    const Instance& instance,
    std::size_t numberOfThreads,
    bool continueIfFailed = false
);
```

The existing requirements for rejecting `numberOfThreads == 0`, bounded worker
creation, indexed future collection, deterministic agent ownership, and
read-only concurrent access to grid state remain unchanged.

### Iterative local-repair solver

Create `include/mapf/solvers/local_path_repair_iterative_solver.hpp` with:

```cpp
class LocalPathRepairIterativeSolver {
public:
    explicit LocalPathRepairIterativeSolver(
        const Instance& instance,
        bool continueIfFailed = false
    );

    LocalPathRepairResult solve();

private:
    const Instance& instance;
    bool continueIfFailed;
};
```

The iterative solver must:

1. start its duration timer before initial planning;
2. allocate one initial-path slot per instance agent;
3. visit agents in instance order on the caller thread;
4. create an `AStarSolver` and run one unconstrained search per agent;
5. attempt all initial searches so every available path can be diagnosed;
6. pass the aligned vector to the shared repair engine; and
7. include initial planning and repair in `metrics.durationSeconds`.

It must not create worker threads. Except for initial-path scheduling, its
returned paths, costs, conflicts, reservations, failure behavior, and metrics
must match the parallel solver for the same initial paths and configuration.

### Priority-planning path access

Keep the existing signature:

```cpp
Result solve(std::list<int> agentsOrder);
```

Add read-only accessors:

```cpp
const std::vector<std::list<Cell*>>& getInitialPaths() const;
const std::vector<std::list<Cell*>>& getPaths() const;
```

The backing path vectors must be private solver state; callers receive only the
const references returned by these accessors.

At the beginning of every `solve` call, reset both stored vectors to the exact
instance-agent count. Calculate and store every unconstrained A* path in
instance order before prioritized planning. Reuse those stored initial paths
when calculating injustice.

Commit each successful prioritized SIPP path immediately into the slot for its
instance agent. If planning fails, leave the failed and not-yet-attempted
solution slots empty while preserving all already committed paths. The public
return type and priority-order validation behavior must remain compatible.

Partial `sumOfCosts`, `makespan`, and `injustice` must describe the stored
non-empty final paths according to the metric rules below instead of being
reset to zero solely because the overall result failed.

## Shared Local-Repair Engine

Create an internal implementation such as:

```text
src/mapf/solvers/local_path_repair_solver_common.hpp
src/mapf/solvers/local_path_repair_solver_common.cpp
```

Expose only to solver implementation files an entry point equivalent to:

```cpp
LocalPathRepairResult repairInitialPaths(
    const Instance& instance,
    std::vector<std::list<Cell*>> initialPaths,
    bool continueIfFailed,
    std::chrono::steady_clock::time_point startedAt
);
```

Move the sequential portions of the parallel solver into this common engine,
including:

- initial/final conflict synchronization;
- deterministic reservation building;
- terminal endpoint checks;
- active-agent and conflict selection;
- path splitting;
- adaptive-window local SIPP repair;
- suffix timing repair;
- candidate validation and commit;
- full origin-to-goal SIPP fallback;
- metric synchronization; and
- final reservation rebuilding.

The public solver classes must contain only their initial-path scheduling logic
and delegation to this common engine. Do not duplicate the repair algorithm or
use subclassing merely to share it.

All behavioral requirements in specification 004 continue to apply to the
extracted code. Extraction must not change conflict order, repair order,
reservation ownership, SIPP semantics, path construction, fallback behavior,
or deterministic results.

### Initial-path failure

If any initial path is empty, do not enter conflict repair. Return all attempted
initial paths, initialize final paths from them, synchronize partial costs,
conflicts, reservations, and duration, and report failure. This behavior is not
affected by `continueIfFailed`.

### Unrepairable conflicts

The shared engine must track conflicts that could not be repaired in the
current path state. Each tracked entry needs a stable fingerprint containing
enough information to distinguish conflict type, timestep, geometry, and the
relevant agent ownership.

When both local repair and full replanning fail:

- if `continueIfFailed == false`, synchronize the result from the current paths
  and return immediately;
- if `continueIfFailed == true`, record the conflict as unresolved for the
  current path-state revision and continue looking for other repairable
  conflicts.

A conflict fingerprint marked unresolved must not be retried while the paths
remain unchanged. Every accepted repair creates a new path-state revision and
invalidates the ignored-fingerprint set because the repair may remove or alter
an earlier unresolved conflict.

Before every return, recompute collision data from the actual final paths. The
returned `remainingConflicts`, normalized records, per-agent success flags, and
conflict CSV must use this recomputed final state, not the historical list of
failed repair attempts. Thus, a formerly unresolved conflict that disappears
as a side effect of another accepted repair must not appear in final output.

An empty instance remains a valid successful solution with zero metrics and
empty vectors, conflicts, and reservations.

## Normalized Final Conflicts

Add a shared diagnostic representation equivalent to:

```cpp
enum class ConflictType { Vertex, Edge };

struct ConflictRecord {
    Cell* cell1;
    Cell* cell2;
    int timestep;
    ConflictType type;
    std::vector<int> agentIds;
};
```

Provide one normalization helper that scans aligned final paths using the same
stay-at-target semantics as `getCollision`. The helper must be the single source
for:

- conflict CSV rows;
- `success_solution_path`; and
- the stats field `paths_resolved`.

Normalization requirements:

1. Ignore empty paths because they occupy no cells.
2. At each timestep, treat a non-empty path as remaining at its final cell
   after its finite list ends.
3. Detect vertex conflicts and non-degenerate opposite-direction edge swaps.
4. Merge pairwise collision events that have the same type, timestep, and
   geometry into one record.
5. Include every participating experiment-local `Agent::id` in the merged
   record, including agents represented through stay-at-goal occupancy.
6. Sort and deduplicate `agentIds`.
7. Canonicalize edge endpoints by coordinates so equivalent edge events receive
   the same key regardless of traversal direction.
8. Sort records by timestep, conflict type, canonical cells, and agent IDs.

The existing `SolutionConflicts` API does not need to gain agent IDs. The
normalized diagnostic type may live in the experiment support layer or another
shared location accessible to solver-result serialization and tests.

## CSV Utility Requirements

In `include/mapf/utils.hpp` and `src/mapf/utils.cpp`, add:

```cpp
using CsvRow = std::vector<std::string>;

bool writeRowsToCsvFile(
    const std::filesystem::path& outputFilename,
    const CsvRow& headers,
    const std::vector<CsvRow>& rows
);
```

The function must:

- accept only a path whose extension is exactly `.csv`;
- reject any row whose field count differs from the header count;
- create all missing parent directories;
- open the target in truncation mode;
- write one header record followed by every row;
- use the existing CSV escaping behavior for delimiters, quotes, and line
  breaks; and
- return true only when the complete stream is in a successful state.

Keep `writeResultsToCsvFile` source compatible and implement it as a one-row
wrapper over `writeRowsToCsvFile`.

## Experiment Result Contract

Normalize solver-specific output before serialization with a structure
equivalent to:

```cpp
struct ExperimentRunResult {
    Result metrics;
    std::vector<std::list<Cell*>> initialPaths;
    std::vector<std::list<Cell*>> solutionPaths;
    std::vector<ConflictRecord> remainingConflicts;
    int numAgents;
    std::string solver;
    bool continueIfFailed;
    bool multithreading;
    std::size_t numThreads;
    bool localRepair;
};
```

The artifact writer must accept an `Instance`, this normalized result, and the
experiment wall-clock time. It must validate `numAgents` as non-negative and
equal to the constructed instance's agent count, validate both path-vector
sizes against that count, and validate `solver` against the three supported
exact names before creating final artifacts. It must derive per-agent flags and
`paths_resolved` itself so solver adapters cannot provide inconsistent values.
Its public contract is equivalent to:

```cpp
bool writeExperimentArtifacts(
    const Instance& instance,
    const ExperimentRunResult& run,
    double experimentTimeSeconds
);
```

### Shared output descriptor

Create all candidate artifact paths at once:

```cpp
struct ExperimentOutputPaths {
    std::string prefix;
    std::filesystem::path directory;
    std::filesystem::path stats;
    std::filesystem::path solution;
    std::filesystem::path conflicts;
};

ExperimentOutputPaths makeExperimentOutputPaths();
```

The prefix format is:

```text
{git_branch}_{timestamp_milliseconds}
```

Use the current Git branch from `.git/HEAD`, preserving the existing
`unknown`/`detached` fallback behavior. Sanitize every character other than an
ASCII letter, digit, `.`, `_`, or `-` to `_`. Use epoch milliseconds and an
in-process monotonic guard so two descriptors created in the same millisecond
cannot resolve to the same prefix.

The final directory and paths must be:

```text
results/{prefix}/
├── {prefix}_stats.csv
├── {prefix}_solution.csv
└── {prefix}_conflicts.csv  # failed local repair only
```

Paths must be rooted at the configured repository root and must not depend on
the process working directory.

### Bundle commit behavior

Write every required artifact inside a temporary per-run directory under
`results/`. Required files are stats and solution for every solver result, plus
conflicts for a failed local-repair result.

Only after every file write succeeds may the writer rename the temporary
directory to `results/{prefix}/`, exposing the completed bundle in one commit.
If any write or rename fails, recursively remove the temporary directory and
any final directory created by that call under the new prefix, then return
false. The cleanup target must be the fully resolved current-run directory; do
not delete artifacts from earlier runs or broader result paths.

Legacy `results/*_results.csv` files are historical data and must not be moved,
rewritten, or deleted.

## CLI Requirements

Implement:

```cpp
int runCli(int argc, char* argv[]);
```

in `src/main.cpp`, and make `main` delegate directly to it.

Accepted syntax:

```text
mapf_app -map <map> -scen <scenario> -solver <solver> -agents <n> [-threads <t>] [-continue_if_failed <true|false>]
```

Flag/value pairs may appear in any order. A path containing spaces works when
the shell supplies it as one argument.

### General parsing rules

- `-map`, `-scen`, `-solver`, and `-agents` are required exactly once.
- Every supplied flag must have a following value.
- Positional arguments, unknown flags, and duplicate flags are errors.
- Parse integers with full-consumption validation, such as `std::from_chars`.
- `-agents` must be a non-negative integer. Zero is valid.
- Solver names are case-sensitive and must match a supported class name
  exactly.
- `-continue_if_failed` accepts exactly `true` or `false` and defaults to
  `false` when absent.
- Reject invalid combinations before constructing the `Instance`.

### Solver-specific validation

| Solver | `-threads` | `-continue_if_failed` | Metadata |
| --- | --- | --- | --- |
| `PriorityPlanningSolver` | rejected | rejected | `multithreading=false`, `num_threads=1` |
| `LocalPathRepairParallelSolver` | required, integer greater than zero | optional | `multithreading=true`, configured thread count |
| `LocalPathRepairIterativeSolver` | rejected | optional | `multithreading=false`, `num_threads=1` |

Unknown solver values must fail validation and must never fall through to the
iterative solver.

### Execution and dispatch

Start the experiment timer immediately before constructing the `Instance` and
stop it immediately after solver completion. The stats `time` field includes
instance loading and solver execution, but excludes artifact serialization.

Dispatch through three explicit branches:

- Priority planning uses the natural instance order `{0, ..., n - 1}`, then
  collects `getInitialPaths()` and `getPaths()`. Its normalized output stores
  `solver=PriorityPlanningSolver` and `continueIfFailed=false`.
- Parallel local repair receives the validated thread count and continuation
  flag. Its normalized output stores
  `solver=LocalPathRepairParallelSolver` and the effective continuation value.
- Iterative local repair receives the continuation flag. Its normalized output
  stores `solver=LocalPathRepairIterativeSolver` and the effective continuation
  value.

Normalize final conflicts from the paths returned by the selected solver. Do
not use a default `else` branch as the iterative dispatch.

Catch parsing, loading, and solver exceptions at the CLI boundary. Print one
actionable error and concise usage text to `stderr`. A failure before a solver
result exists must not create a result bundle.

### Exit codes

- Return `0` only when the solver reports success and every required artifact
  is committed.
- Return `1` when the solver returns an unsuccessful solution or artifact
  writing/commit fails.
- Return `2` for invalid CLI input, instance-loading errors, or an exception
  before a solver result is available.

An algorithmically failed solver still writes its required artifacts before
returning `1`.

## Artifact Schemas

### Statistics CSV

Write exactly one data row with these headers in this order:

```text
map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,continue_if_failed
```

Field rules:

- `map` and `instance_name` use the instance metadata already used by existing
  experiments; benchmark values remain filename-only names.
- `num_agents` is the non-negative integer parsed from the CLI's `-agents`
  flag. The CLI adapter must copy that value into the normalized experiment
  record before serialization.
- `success` is the solver's final boolean result.
- `paths_resolved` is the number of solution rows whose
  `success_solution_path` is true.
- `sumOfCosts` is the sum of costs of every non-empty final path, including
  conflicted paths.
- `makespan` is the greatest cost among non-empty final paths, or `0` if none
  exists.
- `injustice` is calculated only across agents that have both a non-empty
  initial path and a non-empty final path. Use the existing standard-deviation
  definition based on per-agent solution-versus-optimum cost differences. Use
  `0.0` when the eligible set is empty.
- `durationSeconds` is the selected solver's complete solve duration.
- `time` is the CLI experiment duration defined above.
- `multithreading` is true only for `LocalPathRepairParallelSolver`.
- `num_threads` is the configured parallel thread count or `1` for a sequential
  solver.
- `solver` is exactly `PriorityPlanningSolver`,
  `LocalPathRepairParallelSolver`, or `LocalPathRepairIterativeSolver`, matching
  the validated `-solver` CLI value.
- `continue_if_failed` is the effective boolean after applying its default of
  `false`; priority planning always writes `false` because it rejects that CLI
  flag.

Conflicted non-empty paths contribute to costs and injustice but do not count
as resolved. Use the exact name `injustice`, not `justice`.

### Solution CSV

Write one row per instance agent, in instance order, with these headers:

```text
agent_id,agent_scenario_bucket,start,goal,optimum_path,solution_path,optimum_path_cost,solution_path_cost,success_optimum_path,success_solution_path
```

Serialization rules:

- serialize `Agent::scenarioId` under `agent_scenario_bucket`, including
  repeated scenario values and the manual-instance default `-1`;
- serialize a coordinate as `x-y` using the internal `Cell::position` values;
- serialize a path as `x-y|x-y|...` in timestep order;
- retain repeated cells, because they represent waits;
- derive `start` and `goal` from the corresponding instance agent;
- use the aligned initial path for `optimum_path`;
- use the aligned committed final path for `solution_path`;
- serialize an empty path as an empty field with cost `-1`;
- serialize a one-cell path with cost `0`;
- set `success_optimum_path=true` exactly when the initial path is non-empty;
- set `success_solution_path=true` exactly when the final path is non-empty and
  the agent's experiment ID occurs in no normalized final conflict; and
- retain a non-empty conflicted final path and its normal cost for diagnosis,
  but set `success_solution_path=false`.

For file-loaded instances, endpoint and path coordinates must all use the same
coordinate system already produced by `Instance`; no additional coordinate
conversion may be applied during serialization.

A zero-agent experiment creates a header-only solution CSV.

### Conflicts CSV

Create this artifact only when `localRepair == true` and final solver success is
false. Write these headers in order:

```text
cell_1,cell_2,timestep,conflict_type,agents
```

Serialization rules:

- coordinates use `x-y`;
- a vertex conflict writes the same coordinate to `cell_1` and `cell_2` and
  uses `conflict_type=vertex`;
- an edge conflict writes its canonical endpoints and uses
  `conflict_type=edge`;
- `timestep` is the collision timestep;
- `agents` is the sorted, deduplicated list of experiment agent IDs joined by
  `|`; and
- row order follows the deterministic normalized-record order.

If local repair fails because of a missing initial path or another condition
without a geometric final conflict, create a header-only conflicts CSV.

Do not create a conflict artifact for a successful local-repair result or for
any `PriorityPlanningSolver` result.

## Existing Experiment Executables

Update `manual_experiment.cpp` and `benchmark_experiment.cpp` to use the same
normalized artifact writer so the old experiment binaries remain buildable and
follow the new bundle format. These priority-planning executables serialize
`num_agents` equal to the constructed instance's agent count,
`solver=PriorityPlanningSolver`, and `continue_if_failed=false` in stats; manual
agents serialize `agent_scenario_bucket=-1` in solution rows.

The executables must supply aligned initial and solution paths and must apply
the same metric, conflict, and artifact validation rules as the CLI. Remove the
obsolete single-result writer only if no caller still requires it; otherwise
retain a thin compatibility wrapper.

## Required Implementation Sequence

1. Add `Agent::scenarioId` and populate it in file-based instance loading.
2. Move local-repair result and reservation declarations into the neutral
   public header, preserving transitive source compatibility.
3. Extract the sequential repair engine and its helpers from the parallel
   source without changing specification-004 behavior.
4. Add `continueIfFailed`, path-state revision tracking, ignored unresolved
   fingerprints, and final-state conflict recomputation to the shared engine.
5. Adapt the parallel solver to generate aligned initial paths and delegate to
   the common engine.
6. Add the iterative solver and delegate its sequentially generated initial
   paths to the same engine.
7. Extend priority planning with aligned path storage, accessors, and partial
   result metrics.
8. Implement normalized conflict records and use them for all per-agent and
   aggregate resolution decisions.
9. Generalize the CSV utility to rectangular multi-row output while keeping its
   compatibility wrapper.
10. Implement shared output descriptors, artifact serialization, transactional
    bundle commits, and failure cleanup.
11. Replace the demonstration in `src/main.cpp` with strict parsing, explicit
    dispatch, exit codes, timing, and result writing.
12. Migrate the existing experiment executables to the shared writer.
13. Add per-experiment output-directory creation, ignore rules, build targets,
    tests, and documentation.
14. Build, run all tests, execute one smoke run per solver, and inspect the
    emitted schemas and aligned endpoint/path data.

## Repository And Build Changes

### Required source changes

- Update `include/mapf/core/agent.hpp`.
- Update `src/mapf/core/instance.cpp`.
- Create `include/mapf/solvers/local_path_repair_solver.hpp`.
- Update `include/mapf/solvers/local_path_repair_parallel_solver.hpp`.
- Create `include/mapf/solvers/local_path_repair_iterative_solver.hpp`.
- Refactor `src/mapf/solvers/local_path_repair_parallel_solver.cpp`.
- Create `src/mapf/solvers/local_path_repair_iterative_solver.cpp`.
- Create the internal shared local-repair header and implementation.
- Update the priority-planning header and implementation.
- Update `include/mapf/utils.hpp` and `src/mapf/utils.cpp`.
- Update the experiment utility header and implementation.
- Replace `src/main.cpp`.
- Update the manual and benchmark experiment sources.

### Result directories

Keep `results/` as the artifact root. Do not create fixed `stats`, `solution`,
or `conflicts` subdirectories. Create one `{prefix}` subdirectory dynamically
for each experiment and place every artifact for that experiment inside it.

Update `.gitignore` to ignore generated per-experiment directories and their
CSV contents. Preserve all historical root-level `results/*_results.csv` files;
this feature must not move or delete them.

### CMake

- Add the iterative solver and common repair implementation to the `mapf`
  library.
- Link `mapf_app` to `mapf_experiment_utils`.
- Keep the existing `Threads::Threads` dependency.
- Register the new tests with CTest.
- Do not introduce another package dependency.

## Test Requirements

Every scenario-based test must include a source comment describing the
scenario and expected result, consistent with the existing test convention.

### CLI parsing and dispatch

Test that the CLI:

- accepts valid flags in different orders;
- preserves paths containing spaces when passed as one argument;
- accepts zero agents;
- dispatches and records metadata for each of the three solver names;
- rejects missing values, duplicate flags, unknown flags, and positional
  arguments;
- rejects malformed or negative agent counts;
- rejects malformed, zero, or negative thread counts;
- rejects unknown solver values;
- requires `-threads` only for the parallel solver;
- rejects `-threads` for sequential solvers;
- accepts only exact boolean values for `-continue_if_failed`;
- rejects `-continue_if_failed` for priority planning;
- creates no artifact bundle for invalid map or scenario input; and
- returns the documented exit codes.

### Shared and iterative local repair

Test that:

- iterative initial paths equal direct per-agent A* output;
- `initialPaths` remains unchanged after successful and failed repairs;
- iterative and parallel adapters produce equivalent complete results from the
  same initial paths;
- parallel output stays deterministic across supported worker counts;
- `continueIfFailed=false` stops at the first unrepairable conflict;
- `continueIfFailed=true` skips that conflict for the unchanged state and can
  repair later conflicts;
- an unresolved conflict removed by a later repair is absent from final
  conflicts;
- initial-path failure retains every available path and skips repair;
- empty instances, unreachable goals, shared starts, duplicate destinations,
  start-equals-goal paths, repair failures, and both continuation modes behave
  as specified; and
- all pre-existing SIPP, collision, and local-repair tests remain green after
  extraction.

### Priority planning and scenario identity

Test that:

- repeated scenario first-column values are preserved in `scenarioId` while
  `Agent::id` remains unique and sequential;
- manual agents default to `scenarioId=-1`;
- initial and committed paths remain in instance order for a non-natural
  priority order;
- repeated `solve` calls reset stored path vectors;
- failure preserves paths committed before the failed priority item and leaves
  failed/unattempted slots empty; and
- a failure at priority item `X` produces `X - 1` resolved paths independently
  of the numeric IDs.

### CSV and artifact bundles

Test:

- exact header names and order for every schema;
- one solution row per agent and a header-only file for zero agents;
- one `results/{prefix}/` directory per experiment and no fixed artifact-type
  subdirectories;
- exact repetition of the directory prefix in every contained filename;
- distinct prefixes for descriptors created within the same millisecond;
- branch sanitization and repository-root path independence;
- exact `num_agents` values copied from `-agents`, including zero and values
  greater than one;
- exact `agent_scenario_bucket` values in solution rows;
- exact `solver` and `continue_if_failed` values in stats for all three CLI
  dispatch branches and the defaulted continuation case;
- exact path, coordinate, wait, ID, action-cost, and boolean formatting;
- cost `0` for start-equals-goal and `-1` for missing paths;
- stats values for successful and partial failed runs;
- a retained non-empty conflict path marked unsuccessful;
- aggregation and deterministic ordering of vertex and edge conflict records;
- all agent IDs in a conflict involving three or more agents;
- header-only conflict output for a failed local-repair run with no geometric
  conflict;
- omission of conflict output for success and priority failure;
- CSV escaping, extension validation, and row-width validation;
- rejection of misaligned path vectors; and
- cleanup of current-run temporary and, only if created by that call, final
  directories after write or rename failure.

## Documentation Requirements

Create `docs/experiment_cli_and_results.md` documenting:

- complete CLI syntax and validation rules;
- supported solver names and solver-specific flags;
- exit codes;
- output locations and shared-prefix naming;
- all three exact CSV schemas;
- coordinate, wait, and path-cost conventions;
- partial-failure and conflict-file behavior; and
- transactional bundle behavior.

Update `docs/local_path_repair_parallel_solver.md` to describe the neutral
result type, preserved `initialPaths`, shared repair engine,
`continue_if_failed`, unresolved conflict handling, and iterative counterpart.

Update `readme.md` with build and test commands, an example invocation for each
solver, and the per-experiment output-directory layout. At minimum, include the
approved parallel and iterative examples from the plan.

## Validation Commands

Run:

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Also run one real CLI smoke test for each supported solver against a small
checked-in benchmark. Inspect the stats and solution artifacts for schema,
prefix, vector alignment, endpoint consistency, and costs. Run one failing
local-repair fixture and inspect its conflict artifact.

## Acceptance Criteria

- `mapf_app` implements the documented strict CLI and explicit three-solver
  dispatch.
- Invalid input is rejected before instance loading or artifact creation.
- Solver and output failures return the documented exit codes.
- `Agent::scenarioId` preserves the scenario row's first column independently
  of `Agent::id`.
- Parallel and iterative local repair return immutable, aligned
  `initialPaths` and final paths in instance order.
- Both local-repair solvers use one repair implementation and differ only in
  initial-path scheduling.
- `continue_if_failed` defaults to false, uses revision-aware conflict skipping
  when true, and never causes stale conflicts to be serialized.
- Priority planning retains aligned unconstrained and committed paths without
  changing its existing `solve` signature.
- Every run that reaches a solver result creates
  `results/{git_branch}_{timestamp}/` and writes one stats and one solution CSV
  inside it, with both filenames repeating the directory prefix.
- Every failed local-repair run additionally writes a conflict CSV, including a
  header-only file when appropriate, in the same experiment directory.
- No successful run and no priority-planning failure writes a conflict CSV.
- Statistics use the exact requested schema, contain `num_agents` copied from
  `-agents`, the exact selected solver and effective `continue_if_failed`
  value, and retain meaningful partial metrics.
- Solution rows contain `agent_id`, `agent_scenario_bucket`, endpoints, full
  paths including waits, action-count costs, and accurate success flags.
- Conflict rows contain canonical final conflicts and all participating
  experiment-agent IDs in deterministic order.
- `paths_resolved` exactly equals the count of rows with
  `success_solution_path=true`.
- Artifact writes are bundle-safe and clean up only files belonging to their
  own failed commit attempt.
- Existing legacy result files and existing user edits are preserved.
- CMake configuration and compilation succeed, all old and new CTest targets
  pass, and the documented smoke tests produce valid artifacts.

## Out Of Scope

- Arbitrary priority-order selection through the CLI.
- Changing the path representation from `std::list<Cell*>`.
- Changing stay-at-target collision semantics.
- Adding agent IDs to the existing `SolutionConflicts` public structures.
- Introducing a third-party CSV, CLI, or concurrency library.
- Moving or deleting historical `results/*_results.csv` files.
- Providing a cross-process atomic output-name allocator.

## Assumptions And TODOs

- CSV file extensions are explicit because the existing result format and all
  requested artifacts are column-oriented CSV data.
- Sequential solvers report `num_threads=1` even though they do not create a
  worker pool.
- An unsuccessful priority-planning result writes stats and solution artifacts
  but no conflict artifact.
- `success_solution_path` describes both path availability and conflict
  freedom; it is intentionally false for a present but conflicted path.
- `continue_if_failed` affects only conflict repair. It does not make a missing
  initial path or structurally invalid endpoint configuration repairable.

TODO: Add a separate priority-order CLI flag if future experiments need orders
other than natural instance order.

TODO: Replace the in-process timestamp collision guard with an atomic
exclusive-create naming strategy if concurrent processes must write experiment
artifacts to the same repository.
