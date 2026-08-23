Created at: 2026-08-23 10:12:00 -03
Author: lucas
Last updated at: 2026-08-23 10:12:00 -03
AI model used: GPT-5 (Codex)

# Spec: MAPF Priority Planning Experiments

## Objective

Implement MAPF experiments that solve manual and benchmark instances with prioritized planning and save one CSV result file per run under `results/`.

The implementation must complete the missing framework pieces for instances, results, priority planning, CSV writing, and experiment executables. The requirements in the `Adjustments` section of `plan.md` override earlier conflicting plan text.

## Requirements

- Implement the new public headers identified by `git status` under matching `src/` paths.
- Implement `writeResultsToCsvFile` so experiment results can be written to a `.csv` file.
- Create `src/mapf/experiments/manual_experiment.cpp`.
- Create `src/mapf/experiments/benchmark_experiment.cpp`.
- Both experiments must solve through `PriorityPlanningSolver`; they must not call the low-level pathfinding solver directly.
- Each experiment must create an `Instance`, create a `PriorityPlanningSolver`, solve the instance, and then save the results to a CSV file.
- Each result file must be created under `results/`.
- Each result filename must follow `{git_branch}_{timestamp_seconds}_results.csv`.
- Each CSV must contain exactly one header row and one data row.
- The CSV schema must be:

```text
map,instance_name,num_agents,success,sumOfCosts,makespan,injustice,durationSeconds,time
```

- For manual instances, write `map=-` and `instance_name=-`.
- For benchmark instances, write the filename-only map and scenario names.
- The `time` column measures the elapsed experiment time from immediately before instance construction/loading through solver completion. CSV serialization happens afterward.

## Grid And Coordinates

- Remove the Cartesian coordinate-system assumption.
- `Grid` must use top-left origin coordinates: `(0, 0)` is the top-left cell.
- `x` represents the horizontal axis and maps to columns.
- `y` represents the vertical axis and maps to rows.
- `Grid` construction from a matrix must treat `free[y][x]` as the cell at column `x`, row `y`.
- `Grid::getCellPtr(x, y)`, neighbor lookup, Manhattan distance, and grid printing must follow the same top-left-origin coordinate model.
- Moving AI scenario coordinates must be used directly. Do not invert or reverse the Y axis.
- Moving AI `.map` parsing must use `height` as the number of rows and `width` as the number of columns.
- Every `.map` character other than `'.'` must be treated as occupied.

## Public Interfaces

- In `include/mapf/core/agent.hpp`, `Agent::id` must be an `int`.
- `Agent` stores its ID plus current, start, and goal positions by value.
- In `include/mapf/core/instance.hpp`, `Instance` must own its agents with:

```cpp
std::vector<Agent> agents;
```

- The manual `Instance` constructor must receive `std::vector<Agent>` and copy or move those values into owned storage.
- The benchmark `Instance` constructor must load the grid from a Moving AI `.map` file and load the first `numAgents` valid records from the `.scen` file.
- Benchmark-created agents must receive integer IDs in scenario order, starting at `0`.
- `Instance` must expose read-only accessors for agents, grid, dimensions, map name, and instance name. Mutable ownership of agents must not be exposed.
- In `include/mapf/core/result.hpp`, `Result` must have safe default member values and a constructor that receives all attributes:

```cpp
Result(
    bool success,
    int sumOfCosts,
    int makespan,
    double injustice,
    double durationSeconds
);
```

- In `include/mapf/solvers/priority_planning_solver.hpp`, `PriorityPlanningSolver` must receive a read-only reference in its constructor and store a read-only reference attribute:

```cpp
explicit PriorityPlanningSolver(const Instance& instance);
```

- `PriorityPlanningSolver` must not own or copy the `Instance`; the caller must keep the `Instance` alive for the solver lifetime.

## Priority Planning Behavior

- `PriorityPlanningSolver::solve` must process agents in the requested priority order.
- The priority order must use integer agent IDs.
- The solver must reject missing, duplicate, or unknown agent IDs.
- The solver must run SIPP once per agent, passing all previously successful paths as reservations for later agents.
- Earlier agents' planned paths are fixed and become constraints for later agents.
- If any agent cannot be planned, solving fails for the requested priority order.
- A start-equals-goal path is valid if the low-level solver returns a non-empty path.
- An empty path means failure for that priority ordering.
- On success, the complete solution must be validated with the existing MAPF validation behavior.

## Result Metrics

- `success`: true only when every requested agent receives a path and the complete path set is valid.
- `sumOfCosts`: sum of `path.size() - 1` for all agents.
- `makespan`: maximum `path.size() - 1`, or `0` for an empty agent set.
- `injustice`: standard deviation of each agent's path-size increase relative to its unconstrained optimal path.
- To calculate `injustice`, first compute an optimal path for each agent with `AStarSolver`, ignoring conflicts with other agents.
- For each agent, calculate `diff_cost_path` as `solution_path.size() - optimal_path.size()`.
- Calculate the mean of all `diff_cost_path` values.
- Calculate `injustice` as the standard deviation of the per-agent `diff_cost_path` values using that mean.
- `durationSeconds`: wall-clock duration of `PriorityPlanningSolver::solve`, measured with `std::chrono::steady_clock` and populated on both success and failure.
- Failure results must set `success=false` and keep aggregate path metrics zero unless a later requirement explicitly defines partial metrics.

## Experiments

### Manual Experiment

- Create a binary matrix in top-to-bottom row order.
- Create exactly three agents with integer IDs.
- Use valid free start and goal cells.
- Use an explicit arbitrary priority order.
- Create the `Instance`.
- Create the `PriorityPlanningSolver`.
- Solve the instance.
- Save the result row to CSV.
- Return nonzero if solving or CSV writing fails.

### Benchmark Experiment

- Load `benchmarks/maps/empty-8-8.map`.
- Load the first three valid records from `benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen`.
- Use explicit integer IDs for the three generated agents.
- Use an explicit priority order.
- Create the `Instance`.
- Create the `PriorityPlanningSolver`.
- Solve the instance.
- Save the result row to CSV.
- Return nonzero if loading, solving, or CSV writing fails.

## Build And Documentation

- Add the missing implementation files to the `mapf` CMake library target.
- Add separate `manual_experiment` and `benchmark_experiment` executable targets.
- Configure experiment code so benchmark and result paths do not depend on the process working directory.
- Update `readme.md` with commands for building and running both experiments.
- Document that result CSV files are written under `results/`.
- Do not create tests, test files, test executables, or CTest requirements for this feature.

## Acceptance Criteria

- `cmake -S . -B build` succeeds.
- `cmake --build build` succeeds.
- `manual_experiment` builds and runs.
- `benchmark_experiment` builds and runs.
- The manual experiment constructs a binary grid and exactly three agents without reading benchmark files.
- The benchmark experiment loads `empty-8-8.map` and exactly the first three valid records from `empty-8-8-random-1.scen`.
- Both experiments solve through `PriorityPlanningSolver`.
- `Grid` uses top-left-origin coordinates consistently.
- `Agent::id` is an integer.
- `Instance` owns `std::vector<Agent>`.
- `PriorityPlanningSolver` stores `const Instance&`.
- `Result` has a constructor accepting every result attribute.
- Each experiment creates a parseable CSV file under `results/`.
- Each CSV contains exactly the required eight columns in the documented order and one result row.
- No tests are added for this feature.

## Assumptions

- `injustice` uses the standard deviation of each agent's path-size difference compared to its unconstrained optimal path.
- The caller owns the `Instance` lifetime when constructing `PriorityPlanningSolver`.
- Prioritized planning is incomplete by design: `success=false` means the requested priority order failed, not that the MAPF instance is globally unsolvable.
- Existing stay-at-target semantics from `AStarSippSolver` are preserved.

## Adjustments

- Replace the previous Jain-index definition of the metric and rename the metric from `justice` to `injustice`.
- Add `num_agents` to the CSV result file to record how many agents were run in the experiment.
- The adjusted CSV schema is `map,instance_name,num_agents,success,sumOfCosts,makespan,injustice,durationSeconds,time`.
- To calculate `injustice`, first calculate the optimal path for each agent with `AStarSolver`, ignoring conflicts with other agents.
- For each agent, calculate `diff_cost_path` as the difference between the solution path size and the optimal path size.
- Calculate the mean of all `diff_cost_path` values.
- Calculate `injustice` as the standard deviation of the per-agent `diff_cost_path` values using that mean.
