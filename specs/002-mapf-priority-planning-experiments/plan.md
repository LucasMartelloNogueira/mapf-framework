* Created at: 2026-08-21 12:17:56 -03
* Author: lucas
* Last updated at: 2026-08-21 12:17:56 -03
* AI model used: GPT-5 (Codex)

# Plan: MAPF Priority Planning Experiments

## Prompt

### User request

```text
leia o arquvo task.md e faça o que se pede
```

### Task definition from `task.md`

```text
leia o arquivo .github/ai-instructions.md e monte um plano para as seguinte funcionalidades:

descrição: criar implementações de experimentos do MAPF a partir de benchmarks ou instance manuais, salvando dados em um arquivo. A solução do MAPF deve usar a abordagem priority planning


* fazer as implementações em /src dos arquivos novos em /include (usar git status para ver quais arquivos novos devem ser implementados)
* na criação de Instance, um construtor (manual instance) deve criar a grid a partir de uma matriz de 0 e 1, igual ao feito em src/main.cpp e receber uma lista de agentes
* na criação de Instance, um construtor (instance from file) deve criar o grid a partir de um arquivo de benchmark (benchmarks/maps), onde as informações de rows (width) e cols (height) estão no arquivo .map. Todo caracter que não seja "." é uma célula ocupada. O agentes deve ver de um arquivo .scen do mesmo mapa (em benchmarks/scenarios, os arquivos de mapa e as pastas de de cenários tem o mesmo prefixo)
* criar a implementação da função writeResultsToCsvFile que escreve em um arquivo .csv
* criar em src/mapf/experiments dois arquivos: manual_experiment.cpp e benchmark_experiment.cpp. Ambos deve resolver o problema do MAPF usando priority planning (usando o PriorityPlanningSolver). Em manual_experiment.cpp, deve ser usar uma matriz de 0 e 1, com 3 agentes e uma ordem arbitraria de agentes. Em benchmark_experiment.cpp, também deve resolver usando o PriorityPlanningSolver mas lendo do mapa benchmarks/maps/empty-8-8.map e pegando os 3 primeiros agentes de benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen.
* depois de cada experimento, os resultados devem ser escritos em um arquivo .csv. Esses arquivos devem ficar na pasta /results. Os nomes dos arquvios devem ter o formato: "{nome_branch_git}_{timestamp_seconds}_results.csv". as linhas e colunas devem os seguintes dados: 

- map: nome do arquivo de mapa (se mapa vier de benchmark, caso contrário, usar "-")
- instance_name: nome do arquivo de instance (se veio de benchmark, caso contrário usar "-")
- todos os atributos de results.hpp
- time: tempo que o experimento levou para encerrar (se achou solução ou não).
```

## Context

The repository is a C++20 MAPF framework built with CMake. `Grid` represents a four-connected Cartesian grid, `AStarSolver` provides unconstrained shortest paths, and `AStarSippSolver` already plans one agent while respecting vertex and edge reservations created by previously planned agents. This makes `AStarSippSolver` the reusable low-level search for prioritized planning.

The repository context in `.github/mapf.md` defines classical discrete-time MAPF, move/wait actions, vertex conflicts, and edge swaps. The Moving AI benchmark files currently exist under `benchmarks/maps` and `benchmarks/scenarios`, including the exact `empty-8-8` files required by the task.

`git status` identifies three new public headers whose implementations are missing:

- `include/mapf/core/instance.hpp`
- `include/mapf/core/result.hpp`
- `include/mapf/solvers/priority_planning_solver.hpp`

It also identifies the new declaration `writeResultsToCsvFile` in the modified `include/mapf/utils.hpp`. None of these declarations is currently part of the `mapf` CMake target, and there is no test target or `results/` directory.

## Existing Codebase Analysis

### Grid and coordinates

`Grid(std::vector<std::vector<int>>*, int rows, int cols)` already validates rectangular binary matrices. Matrix rows are written top-to-bottom, while `Grid::getCellPtr(x, y)` uses a Cartesian Y axis and reverses matrix row indices during construction.

Moving AI files define `height` and `width`, with `(0, 0)` at the upper-left corner. The implementation must therefore use:

```text
numRows = map height
numCols = map width
internalY = numRows - 1 - scenarioY
```

The task's parenthetical association of rows with width and columns with height conflicts with the established `Grid` contract. Preserving `rows = height` and `cols = width` avoids transposed non-square maps.

### Agents and instance ownership

`Agent` stores an ID plus current, start, and goal positions by value. The draft `Instance` header stores `std::list<Agent*>`, which leaves ownership unclear and makes copies of `Instance` unsafe for agents created by the file constructor. Since `PriorityPlanningSolver` currently receives and stores `Instance` by value, `Instance` should own copied `Agent` values instead of retaining caller pointers.

The public manual constructor can preserve its requested `std::list<Agent*>` input for compatibility but must validate non-null pointers and copy each `Agent` into internal value storage. Public const accessors will let the solver read the grid, agents, and dimensions without exposing mutable ownership.

### Prioritized planning

`AStarSippSolver::solve` accepts already planned paths and constructs safe intervals plus blocked reverse-edge arrivals. `PriorityPlanningSolver::solve` should iterate the requested agent IDs in order and pass all successful earlier paths to SIPP. This is prioritized planning: an earlier agent's path is fixed and becomes a reservation for every later agent.

The solver must reject an order with missing, duplicate, or unknown IDs. A zero-length movement is not a failure because SIPP returns a one-cell path when start equals goal; only an empty path means the current priority ordering cannot be solved.

### Result metrics

`Result` declares `success`, `sumOfCosts`, `makespan`, `injustice`, and `durationSeconds`, but its constructor does not accept `success` and there is no default constructor for `PriorityPlanningSolver::result`. The result interface needs a complete constructor and safe defaults.

Metrics will have these definitions:

- `success`: true only when every requested agent receives a path and the complete path set passes `validateSolution`.
- `sumOfCosts`: sum of `path.size() - 1`, so initial occupancy is not charged as an action.
- `makespan`: maximum `path.size() - 1`, or zero for an empty agent set.
- `injustice`: standard deviation of each agent's path-size increase relative to its unconstrained optimal path.
- `durationSeconds`: wall-clock duration of `PriorityPlanningSolver::solve`, measured with `std::chrono::steady_clock` and populated on success and failure.

The CSV `time` column remains separate: it measures the experiment from immediately before instance construction/loading through solver completion. CSV serialization is performed afterward, so the measured value is available to write.

## Documentation and Libraries Researched

No third-party library is required.

- The Moving AI benchmark format documentation defines `.map` headers as `type`, `height`, `width`, and `map`; `.scen` records contain bucket, map name, map width, map height, start X/Y, goal X/Y, and optimal length. Source: https://www.movingai.com/benchmarks/formats.html
- The Moving AI MAPF benchmark page confirms that each scenario file is an ordered list of start/goal problems and is intended to be consumed by increasing agent count. Source: https://movingai.com/benchmarks/mapf.html
- The C++ standard library provides `std::filesystem::create_directories` for creating `results/` and `std::chrono::steady_clock` for monotonic elapsed-time measurement. Sources: https://www.eel.is/c++draft/fs.op.create.directories and https://www.eel.is/c++draft/time.clock.req

Use only existing C++20 standard-library facilities:

- `<fstream>` and `<sstream>` for strict line-oriented benchmark parsing and CSV output.
- `<filesystem>` for file names, extensions, parent directories, and `results/` creation.
- `<chrono>` for monotonic durations and epoch-second output names.
- `<algorithm>`, `<unordered_map>`, and `<unordered_set>` for agent lookup and order validation.
- `<cmath>` or direct arithmetic for aggregate metrics.
- `<iomanip>` for stable floating-point CSV formatting.

## Files to Change or Create

### Public interfaces

- `include/mapf/core/instance.hpp`
  - Replace non-owning internal pointer storage with owned agent values.
  - Preserve both requested constructor signatures.
  - Add const accessors for the grid, agents, dimensions, and benchmark metadata.
  - Store `mapName` and `instanceName`, using `"-"` for manual instances and filename-only values for benchmark instances.

- `include/mapf/core/result.hpp`
  - Add member initializers representing an unsuccessful zero-valued result.
  - Replace the incomplete constructor with one that accepts every attribute, including `success`.
  - Optionally default the constructor once member initializers make that unambiguous.

- `include/mapf/solvers/priority_planning_solver.hpp`
  - Fix includes to use project-qualified paths.
  - Keep the instance by value or accept it by const reference and copy it once.
  - Initialize `result` safely.
  - Expose the existing `solve(std::list<std::string>)` contract.
  - Keep computed paths local unless a later feature requires returning them.

- `include/mapf/utils.hpp`
  - Add `#pragma once`.
  - Make read-only arguments const references where compatible.
  - Keep `writeResultsToCsvFile` returning `bool`; document that it writes one header and one data row.

### Framework implementations

- `src/mapf/core/instance.cpp`
  - Implement manual matrix and agent construction.
  - Implement strict Moving AI `.map` and `.scen` parsing.
  - Validate file opening, headers, dimensions, row lengths, requested agent count, map/scenario association, coordinate bounds, free start/goal cells, duplicate agent IDs, and sufficient scenario rows.
  - Treat only `'.'` as free, as explicitly required, even though the general Moving AI format documents other traversable terrain symbols.
  - Convert scenario Y coordinates from upper-left origin to the internal Cartesian convention.

- `src/mapf/core/result.cpp`
  - Implement the complete `Result` constructor if it is not defaulted in the header.

- `src/mapf/solvers/priority_planning_solver.cpp`
  - Validate the priority order.
  - Resolve start/goal cells through the instance grid.
  - Run `AStarSippSolver` sequentially with previous paths as reservations.
  - Stop on the first failed agent, retain failure metrics at zero, and always set duration.
  - On success, validate the assembled solution and calculate sum of costs, makespan, and injustice.

- `src/mapf/utils.cpp`
  - Implement `writeResultsToCsvFile`.
  - Reject non-`.csv` targets and mismatched header/value counts.
  - Create the parent directory when needed.
  - Escape fields containing commas, quotes, CR, or LF according to conventional CSV quoting rules, doubling embedded quotes.
  - Open with truncation because each experiment filename identifies one run, write exactly one header and one row, and return the final stream state.

### Experiments

- `src/mapf/experiments/experiment_utils.hpp`
- `src/mapf/experiments/experiment_utils.cpp`
  - Add internal shared helpers for reading and sanitizing the current Git branch, generating epoch-second result paths, serializing booleans/numbers, defining the CSV schema, and writing an experiment row.
  - Read `.git/HEAD` at runtime so switching branches does not require reconfiguring CMake. For a detached HEAD, use a sanitized `detached` label. Accept only letters, digits, `.`, `_`, and `-` in the filename component.
  - Produce `results/{branch}_{timestamp_seconds}_results.csv` exactly.

- `src/mapf/experiments/manual_experiment.cpp`
  - Build a small binary matrix in top-to-bottom row order.
  - Create exactly three agents with unique string IDs and valid free starts/goals.
  - Use a documented arbitrary order that is not dependent on container iteration.
  - Measure construction plus solver execution, then write `map=-` and `instance_name=-` with all `Result` fields and `time`.
  - Return nonzero if solving or CSV writing fails.

- `src/mapf/experiments/benchmark_experiment.cpp`
  - Load `benchmarks/maps/empty-8-8.map` and the first three records from `benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen`.
  - Order the three generated IDs explicitly.
  - Measure loading plus solver execution, then write `map=empty-8-8.map` and `instance_name=empty-8-8-random-1.scen` with all metrics and `time`.
  - Return nonzero if loading, solving, or CSV writing fails.

### Build, tests, and documentation

- `CMakeLists.txt`
  - Add `instance.cpp`, `result.cpp`, `priority_planning_solver.cpp`, and the CSV implementation to the `mapf` library.
  - Add separate `manual_experiment` and `benchmark_experiment` executables.
  - Compile the internal experiment helper into both experiment targets or into a small internal library.
  - Define the repository root for experiment metadata lookup and benchmark paths, avoiding dependence on the process working directory.
  - Enable CTest and add focused test executables without introducing a test framework dependency.

- `tests/core/instance_test.cpp`
  - Cover manual construction, rectangular-map orientation, benchmark parsing, first-N scenario selection, coordinate conversion, and invalid/mismatched files.

- `tests/core/result_test.cpp`
  - Cover default and complete result initialization.

- `tests/solvers/priority_planning_solver_test.cpp`
  - Cover a successful three-agent ordering, an unsolvable ordering, start-equals-goal, invalid order IDs, aggregate metrics, and collision-free output behavior.

- `tests/utils_test.cpp`
  - Cover exact CSV schema output, quote escaping, directory creation, extension rejection, header/value mismatch, and unwritable targets using temporary paths.

- `tests/experiments/experiments_test.cpp`
  - Invoke both experiment executables with a temporary results directory override, assert successful exit, validate the filename pattern, and parse the generated row to confirm all required columns.

- `docs/priority-planning-experiments.md`
  - Document constructor inputs, Moving AI coordinate conversion, prioritized-planning behavior, result metric definitions, CSV schema, build commands, and experiment commands.

- `readme.md`
  - Add commands for the two experiment targets and state where result files are written.

- `.gitignore`
  - Add generated `results/*.csv` while preserving a tracked placeholder if the directory must exist in a clean checkout.

## Proposed Interfaces

### Instance

```cpp
namespace mapf {

class Instance {
private:
    int numRows;
    int numCols;
    std::vector<Agent> agents;
    Grid grid;
    std::string mapName;
    std::string instanceName;

public:
    Instance(
        std::vector<std::vector<int>>* free,
        int rows,
        int cols,
        std::list<Agent*> agents
    );

    Instance(
        std::string mapFilename,
        std::string scenarioFilename,
        int numAgents
    );

    Grid& getGrid();
    const Grid& getGrid() const;
    const std::vector<Agent>& getAgents() const;
    int getNumRows() const;
    int getNumCols() const;
    int getNumAgents() const;
    const std::string& getMapName() const;
    const std::string& getInstanceName() const;
};

}
```

The manual constructor keeps the draft call shape but copies the supplied agents. A follow-up API cleanup can replace pointer input with `const std::vector<Agent>&` without mixing that breaking change into this feature.

### Result

```cpp
namespace mapf {

class Result {
public:
    bool success = false;
    int sumOfCosts = 0;
    int makespan = 0;
    double injustice = 0.0;
    double durationSeconds = 0.0;

    Result() = default;
    Result(
        bool success,
        int sumOfCosts,
        int makespan,
        double injustice,
        double durationSeconds
    );
};

}
```

### Solver flow

```cpp
Result PriorityPlanningSolver::solve(std::list<std::string> agentsOrder) {
    const auto startedAt = std::chrono::steady_clock::now();
    std::vector<std::list<Cell*>> paths;

    // Validate that the order contains every instance agent exactly once.
    for (const std::string& id : agentsOrder) {
        const Agent& agent = findAgentById(id);
        std::list<Cell*> path = sipp.solve(
            instance.getGrid(),
            instance.getGrid().getCellPtr(agent.startPosition.x, agent.startPosition.y),
            instance.getGrid().getCellPtr(agent.goalPosition.x, agent.goalPosition.y),
            paths
        );

        if (path.empty()) {
            return failureMeasuredFrom(startedAt);
        }

        paths.push_back(std::move(path));
    }

    return buildMeasuredResult(paths, startedAt);
}
```

### Map and scenario parsing

```cpp
// After validating `height`, `width`, and `map` headers:
std::vector<std::vector<int>> freeCells(height, std::vector<int>(width));
for (int row = 0; row < height; ++row) {
    std::string mapLine;
    std::getline(mapStream, mapLine);
    require(static_cast<int>(mapLine.size()) == width);

    for (int x = 0; x < width; ++x) {
        freeCells[row][x] = mapLine[x] == '.' ? 1 : 0;
    }
}

// Scenario coordinates use an upper-left origin.
Agent agent {
    .id = std::to_string(index),
    .currentPosition = {startX, height - 1 - startY},
    .startPosition = {startX, height - 1 - startY},
    .goalPosition = {goalX, height - 1 - goalY}
};
```

### CSV writing

```cpp
bool writeResultsToCsvFile(
    const std::string& outputFilename,
    const std::list<std::string>& headers,
    const std::list<std::string>& rowValues
) {
    if (std::filesystem::path(outputFilename).extension() != ".csv" ||
        headers.size() != rowValues.size()) {
        return false;
    }

    std::filesystem::create_directories(
        std::filesystem::path(outputFilename).parent_path()
    );

    std::ofstream output(outputFilename, std::ios::trunc);
    // Write escaped header fields, newline, escaped row fields, and newline.
    return output.good();
}
```

The final CSV schema is:

```text
map,instance_name,num_agents,success,sumOfCosts,makespan,injustice,durationSeconds,time
```

Attribute spellings follow `result.hpp`; `time` is the task's additional end-to-end experiment duration.

## Implementation Sequence

1. Correct and complete the `Instance`, `Result`, solver, and utility public contracts.
2. Implement and unit-test `Result` initialization and strict `Instance` parsing/validation.
3. Implement prioritized planning on top of `AStarSippSolver`, then verify collision handling and metrics.
4. Implement robust single-row CSV output and its filesystem/error cases.
5. Add shared experiment metadata/output helpers.
6. Add the manual three-agent executable and verify it solves using an explicit order.
7. Add the required `empty-8-8` benchmark executable and verify it consumes exactly the first three scenario records.
8. Register all sources, executables, and tests in CMake.
9. Build with warnings enabled for new targets, run CTest, run both experiments, and inspect generated CSV files.
10. Add user-facing documentation and update the README commands.

## Validation and Acceptance Criteria

- `cmake -S . -B build` and `cmake --build build` succeed.
- `ctest --test-dir build --output-on-failure` passes.
- The manual experiment constructs a binary grid and exactly three agents without reading benchmark files.
- The benchmark experiment loads `empty-8-8.map` and exactly the first three valid records of `empty-8-8-random-1.scen`.
- Both executables solve through `PriorityPlanningSolver`; they do not call the low-level solver directly.
- Earlier agents' paths reserve vertices, destination occupancy, and reverse edges for later agents.
- Invalid files, dimensions, coordinates, agent counts, and priority orders fail with actionable errors rather than undefined behavior.
- Success results contain correct sum-of-costs, makespan, injustice, and solver duration; failure results set `success=false` and still record durations.
- Each run creates one parseable file under `results/` named `{sanitized_branch}_{epoch_seconds}_results.csv`.
- Each CSV contains exactly the required eight columns in the documented order and one result row.
- Manual metadata is `-,-`; benchmark metadata is `empty-8-8.map,empty-8-8-random-1.scen`.
- Existing `mapf_app`, A*, SIPP, and solution validation behavior continue to build and run.

## Risks and Explicit Assumptions

- The prompt did not originally define the fairness metric; the adjusted spec defines `injustice` as standard deviation over per-agent path-size increases relative to unconstrained optimal paths.
- The required filename has only one-second resolution and no experiment identifier. Concurrent runs on the same branch during the same second can target the same path. The writer should not silently append a second row; orchestration should run experiments sequentially, or a later requirement should authorize a collision suffix.
- The prompt explicitly says every map character except `'.'` is occupied. This is stricter than Moving AI's general format, where `G` can be traversable, and must be covered by a test to prevent an accidental general-format interpretation.
- Prioritized planning is incomplete by design: failure for one ordering does not prove that the MAPF instance is globally unsolvable. `success=false` means the requested priority order failed.
- `AStarSippSolver` keeps successful earlier agents at their goals forever. This matches the project's current stay-at-target semantics and can make later agents fail when a goal blocks a required route.

TODO: Replace the pointer-based manual `Instance` constructor with a value/reference-based API in a future breaking cleanup.

TODO: Add an explicit run identifier or sub-second component if concurrent experiment execution becomes a requirement.

## Adjustments

- Remove the Cartesian coordinate-system assumption from the plan. Update `Grid` in both header and implementation so `(0, 0)` is the top-left cell of the grid. `x` represents the horizontal axis/columns, and `y` represents the vertical axis/rows. Matrix and Moving AI map coordinates should therefore be used directly as top-left-origin coordinates, without reversing the Y axis.
- In `include/mapf/core/result.hpp`, create a `Result` constructor that receives every result attribute as a parameter.
- Do not create tests for this feature.
- In `include/mapf/core/instance.hpp`, change the owned agent storage to `std::vector<Agent> agents` and update the manual constructor to receive the same type, so `Instance` owns its agents by value.
- In `include/mapf/core/agent.hpp`, change `Agent::id` from `std::string` to `int`.
- In `include/mapf/solvers/priority_planning_solver.hpp`, make `PriorityPlanningSolver` receive a read-only `const Instance&` in its constructor and store a read-only reference attribute.
- When implementing experiment files, use this sequence: create the `Instance`, create the `PriorityPlanningSolver`, solve the instance, then save the results to the CSV file.
- Replace the previous Jain-index definition of the metric. To calculate `injustice`, first compute an unconstrained optimal path for each agent with `AStarSolver`, ignoring conflicts with other agents. Then, for each agent, calculate `diff_cost_path` as the difference between the solution path size and the optimal path size. Calculate the mean of all `diff_cost_path` values. The `injustice` metric is the standard deviation derived from those per-agent path-difference values and their mean, so lower values indicate agents were delayed more evenly relative to their individual unconstrained optimums.
- Rename the `justice` metric to `injustice` in the public result metric and CSV output. Add `num_agents` to the CSV result file, using the adjusted schema `map,instance_name,num_agents,success,sumOfCosts,makespan,injustice,durationSeconds,time`.
