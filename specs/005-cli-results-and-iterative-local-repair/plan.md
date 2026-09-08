* Created at: 2026-09-07 16:59:52 -03
* Author: lucas
* Last updated at: 2026-09-08 15:11:02 -03
* AI model used: GPT-5 (Codex)

# Plan: CLI Experiment Runner, Result Artifacts, and Iterative Local Repair

## Prompt

### User request

```text
leia o arquivo task.md e faça o que se pede
```

### Task definition from `task.md`

```text
leia o arquivo .github/ai-instructions.md e monte um plano para as seguintes funcionalidades:

* criar uma função em src/main.cpp que funciona como uma cli, no qual o usuário passa o mapa, o cenário e o solver como argumentos. Os argumentos devem ter o seguinte formato: {binario} -map {map} -scen {scen} -solver {solver} -agents {agents}. Onde o {binario} é o nome do programa compilado, {map} é nome com path completo do arquivo de mapa (exemplo: benchmarks/maps/den520d.map), {scen} é o nome com path completo do arquivo de cenario (ex: benchmarks/scenarios/den520d/random/den520d-random-1.scen), {solver} é a classe de solver que deve ser utilizada, que pode ser PriorityPlanningSolver ou LocalPathRepairParallelSolver, {agents} é o número de agentes que devem usados no cenário
* criar em /results duas pastas: /stats e /solution. A pasta /stats deve ter os mesmos dados que os arquivos _results tem hoje (map,instance_name,success,sumOfCosts,makespan,justice,durationSeconds,time). Já a pasta /solution deve guardar a solução que foi encontrada em um experimento, ou seja, guardar os dados de todos os agentes de uma instancia. Cada linha tera os dados de um agente. As linhas devem ter as colunas que devem ter os seguintes dados: agent_id: número do id do agente no experimento, agent_scenario_id: número do id do agente que foi lido do arquivo de cenário (número da primeira coluna), start: tupla no formato {x}-{y} da ceula de inicio do agente, goal: tupla no formato {x}-{y} da ceula de destino do agente do agente, optimum_path: caminho ótimo do agente, independente de conflitos no formato {x}-{y}|{x}-{y}|... , solution_path: caminho do agente que foi usado na solução (deve conter as células duplicadas em caso de espera) no formato {x}-{y}|{x}-{y}|..., optimum_path_cost: tamanho do caminho ótimo, solution_path_cost: tamanho do caminho da solução. Para uma mesma instancia, os arquivos de /stats e /solution devem ter o mesmo prefixo, que deve ter o formato: {git_branch}_{timestamp}. Os arquivos da pasta /stats devem ter o nome no formato: {git_branch}_{timestamp}_stats. Os arquivos da pasta /solution devem ter o nome no formato: {git_branch}_{timestamp}_solution.
* os arquivos de /stats devem ter mais duas colunas: multithreading: que significa que o solver usou uma solução com multithreading. E a coluna num_threads: número de threads usadas com multithreading.
* o binário em main deve ter mais uma flag: -threads {t} onde t é o número de threads que será usado em LocalPathRepairParallelSolver
* em include/mapf/solvers e src/mapf/solvers criar um arquivo local_path_repair_iterative_solver, que terá uma classe de solver chamado LocalPathRepairIterativeSolver. Esse solver funciona praticamente igual ao LocalPathRepairParallelSolver mas não usa multhitreading, ele calcula os caminhos inicias ótimos dos agentes de forma sequencial, iterativa. O objetivo do LocalPathRepairIterativeSolver é reutilizar ao máximo do que já foi feito em LocalPathRepairParallelSolver, busque uma estratégia para evitar duplicação de código
* Tanto em LocalPathRepairParallelSolver e LocalPathRepairIterativeSolver deve adicionar em LocalPathRepairResult um campo std::vector<std::list<Cell*>> initialPaths, que irá conter os caminhos ótimos dos agentes, ignorando os conflitos. Esses caminhos que serão usados para gravar dos caminhos dos agentes dos arquivos na pasta /solution
```

## Adjustments

Later numbered adjustments override earlier conflicting requirements.

### Adjustment 1

The following changes from the follow-up task are incorporated throughout this plan:

- Dispatch `LocalPathRepairIterativeSolver` explicitly alongside `PriorityPlanningSolver` and `LocalPathRepairParallelSolver`.
- Use `injustice`, not `justice`, as the statistics CSV column name.
- Add `success_optimum_path` and `success_solution_path` to every per-agent solution row.
- Add `results/conflicts/{git_branch}_{timestamp}_conflicts.csv`. On failed local-repair experiments, it records the conflicts still present at the end, using the columns `cell_1,cell_2,timestep,conflict_type,agents`.
- Add `paths_resolved` to statistics. It counts agents whose final path exists and participates in no remaining conflict.
- Persist partial results on solver failure: statistics, every available initial/final path, accurate per-agent success flags, and the unresolved conflicts for local-repair solvers.
- Add `-continue_if_failed <true|false>` for both local-repair solvers. The default is `false` for backward compatibility. With `false`, repair stops at the first unrepairable conflict; with `true`, that conflict is added to `unresolved_conflicts` and repair continues with other conflicts. The final conflict artifact is based on conflicts that actually remain after all accepted repairs.

### Adjustment 2

* Adjusted at: 2026-09-08 11:39:47 -03

#### User request

```text
faça as seguinte alteraçõe nos arquivos specs/005-cli-results-and-iterative-local-repair/plan.md e specs/005-cli-results-and-iterative-local-repair/spec.md:
* mudar a forma como os arquivos são organizados, para cada experimento: {git_branch}_{timestamp}, crie uma pasta com esse prefixo {git_branch}_{timestamp}. Dentro dessa pasta, pode os arquivos {git_branch}_{timestamp}_statsm {git_branch}_{timestamp}_solution e {git_branch}_{timestamp}_conflicts.
* no arquivo _solution, adicione as colunas: solver: que deve falar qual solver foi utilizado (que deve ser somente os valores PriorityPlanningSolver, LocalPathRepairParallelSolver ou LocalPathRepairIterativeSolver, que vem da flag -solver da cli) e a coluna continue_if_failed: valor da flag -continue_if_failed que vem da cli
* no csv do agentes (_solution), troque o nome agent_scenario_id para agent_scenario_bucket
```

#### Changes

- **Before:** artifacts were separated into the fixed directories `results/stats`, `results/solution`, and `results/conflicts`. **After:** every experiment owns `results/{git_branch}_{timestamp}/`, containing its stats, solution, and conditional conflicts CSVs. The `_statsm` spelling in the request is treated as a typo because this adjustment changes organization rather than the established `_stats` suffix.
- **Before:** solution rows did not identify the selected solver or continuation mode. **After:** every solution row includes `solver` and `continue_if_failed`; `solver` is one of the three exact CLI values and `continue_if_failed` is the effective parsed value, including the default `false`.
- **Before:** the scenario-source column was named `agent_scenario_id`. **After:** it is named `agent_scenario_bucket`, matching the Moving AI scenario field retained by `Agent::scenarioId`.

### Adjustment 3

* Adjusted at: 2026-09-08 12:52:01 -03

#### User request

```text
as colunas solver e continue_if_failed devem estar no arquivo _stats.csv
```

#### Changes

- **Before:** `solver` and `continue_if_failed` were repeated in every per-agent solution row. **After:** both fields are serialized once in the experiment's `_stats.csv` row and are removed from `_solution.csv`.
- The stats schema retains its established column order and appends `solver,continue_if_failed` after `num_threads`.

### Adjustment 4

* Adjusted at: 2026-09-08 15:11:02 -03

#### User request

```text
bote no arquivo _stats a quantidade de agentes que foi usado no experimento na coluna num_agents, deve receber o valor da flag -agents
```

#### Changes

- **Before:** the per-experiment stats CSV omitted `num_agents`. **After:** `num_agents` is restored immediately after `instance_name` and contains the effective value parsed from the CLI's `-agents` flag.
- The normalized experiment record carries the requested agent count explicitly, and the artifact writer verifies that it matches the constructed instance before committing the bundle.

## Objective

Replace the hard-coded demo in `mapf_app` with a validated experiment CLI that loads a Moving AI instance, runs one of the supported solvers, and writes one per-experiment result directory containing statistics and per-agent solution CSVs with the same run prefix. Failed local-repair runs additionally write a conflict CSV in that directory. Add a sequential `LocalPathRepairIterativeSolver` whose conflict-repair behavior is identical to `LocalPathRepairParallelSolver`; only unconstrained initial-path generation differs. Preserve each agent's unconstrained path in `LocalPathRepairResult::initialPaths`, retain partial paths on failure, and expose enough final-conflict information to calculate per-agent success and `paths_resolved` accurately.

The CLI will support all three available experiment choices:

- `PriorityPlanningSolver`
- `LocalPathRepairParallelSolver`
- `LocalPathRepairIterativeSolver`

## Existing Codebase Analysis

### Current executable and instance loading

`src/main.cpp` is a hard-coded two-agent demonstration and accepts no arguments. `Instance(mapFilename, scenarioFilename, numAgents)` already validates and loads Moving AI map/scenario files and selects the first `numAgents` non-empty scenario rows, so the CLI should construct an `Instance` directly instead of duplicating parsing.

The first scenario column is currently parsed into a local variable named `bucket` and discarded. `Agent::id` is instead assigned sequentially from zero. These are different concepts, and the sample scenarios contain duplicate first-column values. Add a separate `Agent::scenarioId` field so:

- `agent_id` remains the unique experiment-local ID used by solvers;
- `agent_scenario_bucket` is serialized from `Agent::scenarioId` and preserves the scenario file's first column exactly, even when repeated.

Manual instances have no scenario-row identifier and will use `scenarioId = -1`. Give this field a default initializer so existing designated `Agent` initializers remain source compatible.

### Solver result shapes

`LocalPathRepairParallelSolver` returns paths and metrics, but its initial unconstrained paths are overwritten as repairs are committed. `PriorityPlanningSolver` returns only `Result`; both its prioritized solution paths and the unconstrained paths calculated for injustice are local variables. The artifact writer therefore cannot currently produce the requested solution rows, partial-failure diagnostics, per-agent success flags, or `paths_resolved` for every CLI solver.

Keep `PriorityPlanningSolver::solve(std::list<int>) -> Result` for compatibility and add read-only `getPaths()` and `getInitialPaths()` accessors. Both collections will be stored in instance-agent order, independent of priority order. Calculate and retain every unconstrained initial path before prioritized planning begins. During prioritized planning, commit each successful conflict-free path immediately and leave the failed and not-yet-attempted entries empty if planning stops. Consequently, a failure at the Xth item in the requested priority order yields `paths_resolved = X - 1`, regardless of the numeric agent IDs. The CLI will use the natural instance order `{0, ..., n-1}` unless a future feature adds an explicit priority-order option.

`LocalPathRepairResult` is shared by parallel and iterative local repair and should be defined once in a neutral local-repair header. Add:

```cpp
std::vector<std::list<Cell*>> initialPaths;
```

with the invariant that `initialPaths[i]`, `paths[i]`, and `instance.getAgents()[i]` refer to the same agent. Copy or move the initial vector into `initialPaths` before any path repair, and initialize `paths` from it.

`SolutionConflicts` currently identifies conflict geometry and time but not the participating agents, and it may contain duplicate geometry/time entries generated from different agent pairs. Add a diagnostic normalization helper that scans the final aligned paths and creates one record for each unique conflict:

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

The records use experiment-local `Agent::id` values, sorted and deduplicated. They are the shared source for the conflict CSV, `success_solution_path`, and `paths_resolved`, avoiding inconsistencies between artifacts.

### Reusing local repair

Almost all of `local_path_repair_parallel_solver.cpp` is sequential repair logic. Only the initial A* phase and the private thread pool are parallel. Extract the reservation builder, conflict selection, adaptive-window repair, suffix handling, validation, metric synchronization, and full replanning into an internal shared implementation. The two public solvers will only select how to generate initial paths:

```cpp
LocalPathRepairResult repairInitialPaths(
    const Instance& instance,
    std::vector<std::list<Cell*>> initialPaths,
    bool continueIfFailed,
    std::chrono::steady_clock::time_point startedAt
);
```

`LocalPathRepairParallelSolver` will retain its bounded pool and indexed futures. `LocalPathRepairIterativeSolver` will run one `AStarSolver` call per agent in instance order on the caller thread. Both then call `repairInitialPaths`, guaranteeing identical repair, failure, reservation, continuation, and metric semantics without subclassing or exposing implementation details.

The shared repair engine keeps a collection named `unresolved_conflicts` plus a set of fingerprints tied to the current path-state revision. The conflict selector accepts that set and chooses the next conflict not already declared unresolved for the unchanged state. When a local repair and its full-replanning fallback both fail:

- with `continueIfFailed == false`, synchronize the result and return immediately;
- with `continueIfFailed == true`, append the conflict to `unresolved_conflicts`, record its stable fingerprint, skip retrying it against the unchanged path state, and continue searching for repairable conflicts involving the current and subsequent agents.

Every accepted repair advances the path-state revision and invalidates the ignored-fingerprint set because it may remove or change a previously unresolved conflict. Before returning, recompute conflicts from the final paths and use that final set as `LocalPathRepairResult::remainingConflicts` and as the source of normalized artifact records. A run is successful if all agents have paths and this final set is empty, even if a conflict previously appended to `unresolved_conflicts` disappeared as a side effect of another repair.

### Current result writing

`experiment_utils` currently creates one `results/{branch}_{epoch-seconds}_results.csv` file. `writeResultsToCsvFile` creates parent directories and performs RFC-style CSV escaping, but accepts exactly one row. The solution and conflict artifacts need multiple rows, so add a multi-row CSV function and retain the existing one-row function as a compatibility wrapper.

Create the output directory and all output paths from one run descriptor so separate timestamp calls cannot produce mismatched prefixes:

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

Use a sanitized Git branch and an epoch-millisecond timestamp. Milliseconds still satisfy `{git_branch}_{timestamp}` while reducing accidental overwrite risk for experiments started within one second. Every experiment gets a directory with that prefix, and all artifacts use `.csv`, matching the existing result format:

```text
results/{git_branch}_{timestamp}/
├── {git_branch}_{timestamp}_stats.csv
├── {git_branch}_{timestamp}_solution.csv
└── {git_branch}_{timestamp}_conflicts.csv  # failed local repair only
```

Stats and solution are written for every run that reaches a solver result. The conflicts file is written only for a failed local-repair run; it may be header-only when failure was caused by a missing initial path and no geometric conflict remains. Successful runs and `PriorityPlanningSolver` failures do not create a conflicts file because no failed local repair has unresolved conflicts to report.

Generated legacy `results/*_results.csv` files are historical data and must not be moved or deleted by this feature.

### Baseline

The current working tree builds successfully with `cmake --build build -j2`. All three registered CTest tests pass. Existing user changes are limited to explanatory comments in the SIPP and parallel-repair implementation/header and must be preserved during implementation.

## Documentation and Libraries Researched

No new third-party dependency is needed.

- The C++20 `std::from_chars` integer overload provides allocation-free parsing with explicit error and full-consumption checks, suitable for `-agents` and `-threads`: https://eel.is/c++draft/charconv
- `std::filesystem::create_directories` creates every missing component and treats an already existing directory as success, matching the per-experiment directory under `results/`: https://eel.is/c++draft/fs.op.create.directories
- RFC 4180 describes conventional comma-separated records, quoting, and doubled embedded quotes; the current escaping logic already follows those rules: https://www.rfc-editor.org/info/rfc4180/

Use the existing C++20 standard library:

- `<charconv>` for strict integer CLI parsing;
- `<filesystem>`, `<fstream>`, and `<sstream>` for shared-prefix artifact paths and CSV serialization;
- `<chrono>` for elapsed time and the shared timestamp;
- `<thread>`, `<future>`, and the existing synchronization primitives only in the parallel initial-planning adapter;
- `<variant>` is unnecessary because the CLI can normalize each solver's result into one small internal experiment record.

## Public Interfaces and Data Contracts

### Agent scenario identity

Extend `Agent` without changing the meaning of `id`:

```cpp
struct Agent {
    int id;
    int scenarioId = -1;
    Position currentPosition;
    Position startPosition;
    Position goalPosition;
};
```

The file-based `Instance` constructor assigns `.scenarioId = bucket`. Manual construction preserves an explicitly supplied value or the default `-1`.

### Shared local-repair result

Move `VertexAgents`, `GoalReservations`, `PathReservationState`, and `LocalPathRepairResult` from the parallel-only header to `include/mapf/solvers/local_path_repair_solver.hpp`. Both solver headers include it. The resulting contract is:

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

Add the same final constructor parameter and default to the parallel solver:

```cpp
LocalPathRepairParallelSolver(
    const Instance& instance,
    std::size_t numberOfThreads,
    bool continueIfFailed = false
);
```

The default preserves existing callers and tests. The option controls only conflict-repair failure; initial-path failure and structurally invalid instances still terminate because no meaningful repair pass can proceed.

Moving the shared declarations must remain source compatible for callers that include `local_path_repair_parallel_solver.hpp`, because that header will transitively include the neutral header.

### Priority-planning path access

Add private stored path vectors plus const accessors:

```cpp
const std::vector<std::list<Cell*>>& getInitialPaths() const;
const std::vector<std::list<Cell*>>& getPaths() const;
```

Reset both vectors at the start of every `solve` call. `initialPaths` contains independent A* paths for every instance agent. `paths` contains the committed prioritized paths in instance order; missing paths remain empty on failure. Use `initialPaths` directly to calculate injustice instead of rerunning A* and discarding the result.

### CSV writer

Add a general overload using a rectangular collection:

```cpp
using CsvRow = std::vector<std::string>;

bool writeRowsToCsvFile(
    const std::filesystem::path& outputFilename,
    const CsvRow& headers,
    const std::vector<CsvRow>& rows
);
```

Reject a non-`.csv` path or any row whose width differs from the header. Create the parent directory, truncate a run-specific target, write one header followed by all rows, and return the final stream state. Keep `writeResultsToCsvFile` and implement it through this function.

### Experiment artifact input

Normalize solver-specific return values before serialization:

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

bool writeExperimentArtifacts(
    const Instance& instance,
    const ExperimentRunResult& run,
    double experimentTimeSeconds
);
```

The writer creates one per-experiment directory and all three candidate paths from one prefix. It validates that `numAgents` is non-negative and matches the constructed instance, validates that `solver` is one of the three supported exact names, and derives both per-agent success flags and `paths_resolved` from `initialPaths`, `solutionPaths`, and normalized `remainingConflicts`; solver code must not calculate separate competing values. To avoid exposing an incomplete artifact bundle, write every required CSV inside a temporary per-run directory under `results/`, then rename that directory to its final `{git_branch}_{timestamp}` name. If any required write or rename fails, remove the current call's temporary directory and any final directory already committed under that newly generated prefix, then return `false`.

## CLI Behavior

Implement `int runCli(int argc, char* argv[])` in `src/main.cpp`; `main` delegates directly to it. Accept flag/value pairs in any order:

```text
mapf_app -map <map> -scen <scenario> -solver <solver> -agents <n> [-threads <t>] [-continue_if_failed <true|false>]
```

Validation rules:

1. `-map`, `-scen`, `-solver`, and `-agents` are mandatory and may appear once.
2. `-agents` is a fully parsed non-negative integer. Zero agents remain valid because the solvers already define an empty solution.
3. `-solver` accepts the three exact class names listed in the objective. Unknown values fail with usage text.
4. `-threads` is mandatory and strictly positive for `LocalPathRepairParallelSolver`; it is rejected for the two sequential solvers so ignored configuration cannot contaminate experiment metadata.
5. `-continue_if_failed` accepts exactly `true` or `false`, is optional with default `false`, and is accepted only for `LocalPathRepairParallelSolver` and `LocalPathRepairIterativeSolver`. Reject it for `PriorityPlanningSolver` rather than silently ignoring it.
6. Unknown flags, missing values, duplicates, and positional arguments fail before loading files.
7. Catch input/loading/solver exceptions at the CLI boundary, print one actionable error plus usage to `stderr`, and do not create result artifacts when the experiment never reaches a solver result.

Dispatch examples:

```cpp
if (options.solver == "PriorityPlanningSolver") {
    PriorityPlanningSolver solver(instance);
    Result metrics = solver.solve(instanceAgentIds(instance));
    run = {
        metrics,
        solver.getInitialPaths(),
        solver.getPaths(),
        normalizeConflicts(instance, solver.getPaths()),
        options.agents,
        "PriorityPlanningSolver",
        false,
        false,
        1,
        false
    };
} else if (options.solver == "LocalPathRepairParallelSolver") {
    LocalPathRepairParallelSolver solver(
        instance,
        options.threads.value(),
        options.continueIfFailed
    );
    LocalPathRepairResult result = solver.solve();
    run = {
        result.metrics,
        result.initialPaths,
        result.paths,
        normalizeConflicts(instance, result.paths),
        options.agents,
        "LocalPathRepairParallelSolver",
        options.continueIfFailed,
        true,
        *options.threads,
        true
    };
} else if (options.solver == "LocalPathRepairIterativeSolver") {
    LocalPathRepairIterativeSolver solver(instance, options.continueIfFailed);
    LocalPathRepairResult result = solver.solve();
    run = {
        result.metrics,
        result.initialPaths,
        result.paths,
        normalizeConflicts(instance, result.paths),
        options.agents,
        "LocalPathRepairIterativeSolver",
        options.continueIfFailed,
        false,
        1,
        true
    };
} else {
    throw std::invalid_argument("Unknown solver: " + options.solver);
}
```

The final explicit branch is deliberate: every supported solver has its own named dispatch path, and the fallback cannot accidentally treat a future or misspelled solver as iterative local repair.

Start the experiment timer immediately before `Instance` construction and stop it after solver completion, preserving the current meaning of `time`. Write artifacts for both successful and algorithmically unsuccessful solver results so failure metrics, partial paths, and final conflicts are retained. Return `0` only when solving succeeds and every required artifact is committed; return `1` for an unsuccessful solution or output failure and `2` for CLI/input exceptions.

Update the README with these commands:

```bash
./build/mapf_app \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 \
  -threads 8 \
  -continue_if_failed true

./build/mapf_app \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairIterativeSolver \
  -agents 100 \
  -continue_if_failed false
```

## Artifact Schemas

### Statistics CSV

Write exactly one row with these headers in order:

```text
map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,continue_if_failed
```

`num_agents` contains the non-negative integer parsed from the `-agents` CLI flag. `injustice` serializes the existing `Result::injustice` value using the same name as the internal metric. `paths_resolved` is derived from the per-agent rule below, not from an agent ID or the total number of non-empty paths. `multithreading` is `true` only for `LocalPathRepairParallelSolver`. `num_threads` records the configured `-threads` value for that solver and `1` for the two sequential solvers. `solver` contains exactly `PriorityPlanningSolver`, `LocalPathRepairParallelSolver`, or `LocalPathRepairIterativeSolver`, matching the validated `-solver` value. `continue_if_failed` contains the effective boolean CLI value after applying the default `false`; priority planning therefore serializes `false` because that solver rejects the flag.

On failure, `sumOfCosts` and `makespan` are calculated over the non-empty final paths that are saved, rather than reset to zero. Calculate `injustice` over agents that have both a non-empty optimum path and a non-empty final path; use `0.0` when that set is empty. Conflicted final paths remain part of the cost/fairness metrics, while `paths_resolved` states how many are actually conflict-free. This makes the failure statistics describe the same partial result as the solution CSV without conflating path availability with conflict resolution.

The CLI adapter copies `options.agents` into the normalized experiment record instead of recomputing the value during serialization. Non-CLI experiment executables use their constructed instance's agent count for the same field.

### Solution CSV

Write one row per instance agent, in `Instance::getAgents()` order, with these headers:

```text
agent_id,agent_scenario_bucket,start,goal,optimum_path,solution_path,optimum_path_cost,solution_path_cost,success_optimum_path,success_solution_path
```

Serialization rules:

- `agent_scenario_bucket` contains `Agent::scenarioId`, including repeated bucket values and the manual-instance default `-1`;
- coordinates use the framework's Cartesian `Cell::position` as `x-y`;
- paths join every cell with `|`, including repeated cells for waits;
- path cost means number of actions, `path.size() - 1`, consistent with `Result`, `pathCosts`, sum of costs, and makespan;
- a one-cell start-equals-goal path has cost `0`;
- a missing path on solver failure is an empty field with cost `-1`, distinguishing failure from a valid zero-cost path;
- `optimum_path` comes from unconstrained A* and `solution_path` comes from the solver's committed output;
- `success_optimum_path` is `true` exactly when the aligned unconstrained path exists, including a one-cell start-equals-goal path;
- `success_solution_path` is `true` exactly when the aligned final path exists and that agent participates in no normalized final vertex or edge conflict;
- a non-empty final path that remains conflicted is serialized for diagnosis with its normal cost, but has `success_solution_path=false` and does not count toward `paths_resolved`;
- `paths_resolved` is the number of solution rows with `success_solution_path=true`;
- all path vectors must be aligned with agent count before serialization; a size mismatch is an artifact-writing error rather than silently truncating rows.

The file-based `Instance` constructor converts scenario coordinates to the grid's Cartesian coordinate system. Both `start`/`goal` and path fields use those same internal coordinates so every endpoint matches its serialized path.

### Conflicts CSV

For a failed local-repair experiment, write one row per unique conflict still present in the final paths, with these headers:

```text
cell_1,cell_2,timestep,conflict_type,agents
```

Serialization rules:

- coordinates use `x-y`, consistently with the solution CSV;
- for a vertex conflict, `cell_1` and `cell_2` both contain the conflicted cell and `conflict_type` is `vertex`;
- for an edge conflict, the two fields contain the edge endpoints in the same deterministic coordinate order used by collision detection and `conflict_type` is `edge`;
- `timestep` is the conflict time reported by collision detection;
- `agents` contains all experiment-local agent IDs involved in that unique conflict, sorted ascending and joined with `|` (for example, `0|3|8`);
- conflict records are sorted deterministically by timestep, type, cells, and agent IDs;
- detect participants using the framework's stay-at-goal semantics after a path ends;
- a failed local-repair run with no geometric conflict produces a header-only file; no conflicts CSV is produced for a successful run.

## Implementation Changes by Subsystem

### Core model and solvers

- Update `include/mapf/core/agent.hpp` and `src/mapf/core/instance.cpp` to preserve the scenario-row identifier separately from the unique experiment agent ID.
- Create `include/mapf/solvers/local_path_repair_solver.hpp` for shared result/reservation types and update the parallel header to include it.
- Create `include/mapf/solvers/local_path_repair_iterative_solver.hpp` and `src/mapf/solvers/local_path_repair_iterative_solver.cpp` for sequential initial A* planning.
- Create an internal `src/mapf/solvers/local_path_repair_solver_common.hpp/.cpp`; move the repair engine and shared helpers out of the parallel implementation. Leave the thread pool and future collection in the parallel source.
- Update both local-repair adapters to populate `initialPaths` before calling the common repair engine.
- Add `continueIfFailed` with a default of `false` to both local-repair constructors and implement the shared unresolved-conflict continuation policy in the common repair engine.
- Add a normalized conflict-record helper that identifies every participating agent, deduplicates pairwise collision output, and recomputes the final records after repairs.
- Update `PriorityPlanningSolver` to retain aligned initial/final paths, partial failure results, and meaningful partial metrics, and expose const getters without changing its existing `solve` return type.

### CLI and output

- Replace the hard-coded `src/main.cpp` demo with `runCli`, strict option parsing, explicit three-solver dispatch, `-continue_if_failed`, normalized result construction, artifact writing, exit codes, and concise usage text.
- Extend `include/mapf/utils.hpp` and `src/mapf/utils.cpp` with rectangular multi-row CSV output while keeping the one-row wrapper.
- Replace the single-result path/writer API in `src/mapf/experiments/experiment_utils.hpp/.cpp` with one shared run prefix and stats/solution/optional-conflicts serializers. Keep a thin compatibility wrapper only if existing callers still need it during the transition.
- Update `manual_experiment.cpp` and `benchmark_experiment.cpp` to use the same artifact writer; manual agent rows serialize `agent_scenario_bucket=-1`, while their stats rows serialize `solver=PriorityPlanningSolver` and `continue_if_failed=false`.
- Keep `results/` as the artifact root, create one dynamically named subdirectory per experiment, and update `.gitignore` to ignore generated per-experiment directories without deleting legacy root-level result CSVs.

### Build and documentation

- Add the iterative and shared repair sources to the `mapf` library in `CMakeLists.txt`.
- Link `mapf_app` against `mapf_experiment_utils`, since CLI artifact generation belongs to the existing experiment support layer.
- Add new test targets to CTest and keep the existing `Threads::Threads` link; no additional package lookup is needed.
- Add `docs/experiment_cli_and_results.md` covering flags, solver choices, exit codes, the per-experiment directory layout, all three schemas including the new stats metadata columns, coordinate/cost conventions, partial-failure output, and artifact-bundle behavior.
- Update `docs/local_path_repair_parallel_solver.md` to describe `initialPaths`, the shared repair engine, `continue_if_failed`, unresolved conflicts, and the iterative counterpart. Update `readme.md` with build, test, three-solver CLI examples, and the per-experiment output layout.

## Test Plan

### CLI parsing and dispatch

- Accept every flag order and paths containing spaces when passed as one argument.
- Reject missing/duplicate/unknown flags, missing values, malformed or negative agent counts, zero/negative thread counts, malformed boolean values, an unknown solver, `-threads` missing for parallel, `-threads` present for a sequential solver, and `-continue_if_failed` present for priority planning.
- Run a small benchmark through each solver and assert `num_agents` equals the `-agents` value, the selected solver metadata in stats, exit code, stats/solution artifacts, and conditional conflict artifact.
- Verify invalid map/scenario paths report an error and create no new final artifact bundle.
- Add a source-code comment to every scenario test describing its scenario and expected behavior, following the convention of the existing tests.

### Iterative and shared local repair

- Confirm iterative initial paths equal direct per-agent A* paths and are stored unchanged in `initialPaths` after repairs.
- Run the existing local-repair fixtures through both public solvers and assert equal success/failure, initial paths, repaired paths, costs, conflicts, and reservations.
- Confirm parallel results are deterministic across supported worker counts and that `initialPaths` remains in instance order.
- Run an unrepairable-conflict fixture with `continueIfFailed=false` and verify immediate termination plus retained paths and final conflicts.
- Run a fixture containing both unrepairable and repairable conflicts with `continueIfFailed=true`; verify the solver skips the unresolved fingerprint, continues repairing other conflicts, and reports only conflicts that remain in the final paths.
- Verify a conflict initially marked unresolved but eliminated by a later accepted repair is absent from the final conflict records.
- Cover empty instances, unreachable initial paths, shared starts, duplicate permanent goals, repair failure, and start-equals-goal for both adapters and both continuation modes where applicable.
- Keep all existing SIPP, collision, and parallel-repair tests passing after helper extraction.

### Priority paths and scenario IDs

- Parse scenario rows with repeated first-column values and assert unique sequential `Agent::id` values plus faithfully retained `scenarioId` values.
- Verify manual agents default to `scenarioId=-1`.
- Assert `PriorityPlanningSolver` exposes both initial and solution paths in instance order even when planning order differs.
- Verify repeated calls reset stored paths and a failed call preserves paths committed before the failure while leaving the failed and unattempted final paths empty.
- Verify a priority failure at the Xth planning item yields `paths_resolved = X - 1`, independent of agent ID values.

### CSV artifacts

- Verify one per-experiment directory is created, its name is the sanitized branch/timestamp prefix, and every artifact inside it repeats that exact prefix.
- Assert the exact stats header/order, including `num_agents`, `paths_resolved`, and `injustice`, and values for threaded and sequential runs.
- Assert one solution row per agent, the exact `agent_scenario_bucket`, exact `x-y|...` formatting, retained wait duplicates, action costs, and both success flags. Assert the exact `solver` and `continue_if_failed` values in the single stats row.
- Verify a non-empty conflicted path is retained with `success_solution_path=false`, while an existing conflict-free path is `true`, and that their total matches `paths_resolved`.
- Assert exact conflict headers, vertex same-cell serialization, deterministic edge endpoint ordering, timestep, sorted pipe-separated agent IDs, aggregation of three-or-more-agent vertex conflicts, and stable row order.
- Verify failed local repair creates a conflict CSV, including the header-only no-geometric-conflict case, while success and priority-planning failure do not.
- Cover start-equals-goal (`0`), missing paths (`-1`), zero agents (header-only solution CSV), and mismatched vector sizes.
- Cover CSV escaping, row-width validation, `.csv` extension validation, write/rename failure, and cleanup of every current-run temporary/final artifact.
- Verify two quickly generated output descriptors have distinct millisecond timestamps; if the clock value is equal, advance deterministically within the process to avoid overwriting.

### Validation commands

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Run at least one real CLI smoke test for each solver with a small checked-in benchmark and inspect stats/solution CSVs for endpoint/path alignment. Also run one failing local-repair smoke fixture and inspect its conflicts CSV.

## Acceptance Criteria

- `mapf_app` accepts the documented flags, validates them before dispatch, and supports all three solver class names.
- Solver dispatch has explicit branches for `PriorityPlanningSolver`, `LocalPathRepairParallelSolver`, and `LocalPathRepairIterativeSolver`; an unknown value never falls through to iterative repair.
- `-threads` controls `LocalPathRepairParallelSolver`; sequential solvers report `multithreading=false` and `num_threads=1`.
- `-continue_if_failed` controls both local-repair solvers, defaults to `false`, and is rejected for priority planning.
- Each run that reaches a solver result creates `results/{git_branch}_{timestamp}/` with exactly one stats CSV and one solution CSV carrying that same prefix. A failed local-repair run also creates its conflicts CSV in that directory.
- Stats contain the exact requested fields, including `num_agents` copied from `-agents`, `injustice`, `paths_resolved`, `multithreading`, and `num_threads`.
- Statistics output contains the selected `solver` and effective `continue_if_failed` value once per experiment.
- Solution output contains one aligned row per agent with `agent_id`, `agent_scenario_bucket`, endpoints, unconstrained path, committed solution path, waits, consistent costs, and accurate `success_optimum_path`/`success_solution_path` values, including on failure.
- Conflict output contains the normalized final vertex/edge conflicts and all involved agent IDs in the requested schema.
- Scenario first-column values survive file parsing independently of unique experiment agent IDs.
- Both local-repair solvers return unchanged `initialPaths`, and all result vectors use instance-agent order.
- Iterative and parallel local repair use one shared repair engine, share immediate/continuing failure behavior, and differ only in initial-path scheduling.
- `PriorityPlanningSolver` exposes the initial and committed paths needed by the writer without breaking its existing `solve` signature.
- Existing user edits remain intact, all old tests pass, and the new CLI, artifact, instance, priority, and iterative tests pass.
- Documentation explains commands, schemas, conventions, output locations, and failure behavior.

## Explicit Assumptions and TODOs

- File extensions are `.csv`, inferred from the existing `_results.csv` artifacts and the column-oriented requirements.
- The new iterative solver is included in CLI dispatch even though the first CLI bullet predates its introduction later in the task.
- Path costs are action counts (`size - 1`), matching the framework's existing metric semantics.
- `num_threads` records the configured parallel thread count; sequential execution records `1`.
- `-continue_if_failed` defaults to `false` and accepts explicit `true`/`false` values; the follow-up task did not specify a different default or boolean syntax.
- A conflict CSV is specific to failed local-repair runs, because the requested contents describe failed repair attempts. Stats and solution are still written for priority-planning failures.
- `success_solution_path` means the path exists and the agent is absent from all final conflicts; merely having a non-empty but conflicted path is not considered resolved.
- Output is retained for algorithmic failure but not for argument, input-file, or construction errors where no solver result exists.

TODO: If future experiments need arbitrary priority orders, add a separate CLI option and keep solution serialization in instance-agent order.

TODO: If several processes write results simultaneously, replace the in-process timestamp collision guard with an atomic exclusive-create naming strategy.
