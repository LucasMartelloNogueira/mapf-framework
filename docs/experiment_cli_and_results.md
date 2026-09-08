# Experiment CLI And Result Artifacts

`mapf_app` loads a Moving AI map/scenario pair, runs one MAPF solver, and writes
one self-contained result directory.

## Command Line

```text
mapf_app -map <map> -scen <scenario> -solver <solver> -agents <n> [-threads <t>] [-continue_if_failed <true|false>]
```

The required flags are `-map`, `-scen`, `-solver`, and `-agents`. Flag/value
pairs may appear in any order and each flag may occur only once. `-agents` is a
non-negative integer, so a zero-agent experiment is valid.

Supported solver names and options are:

| Solver | `-threads` | `-continue_if_failed` |
| --- | --- | --- |
| `PriorityPlanningSolver` | rejected | rejected |
| `LocalPathRepairParallelSolver` | required and greater than zero | optional, default `false` |
| `LocalPathRepairIterativeSolver` | rejected | optional, default `false` |

The continuation flag accepts only the exact values `true` and `false`. When it
is false, local repair stops at the first conflict that neither local repair nor
full replanning can solve. When true, that conflict is skipped while the paths
remain unchanged so the solver can attempt other conflicts. Final results
always describe conflicts that actually remain, not historical repair
failures.

## Exit Codes

- `0`: solving succeeded and the complete artifact directory was committed;
- `1`: solving returned an unsuccessful result or artifact writing failed;
- `2`: CLI validation, instance loading, or solver execution failed before a
  result was available.

An algorithmic failure still writes its diagnostic artifacts. Invalid CLI or
input files do not create an experiment directory.

## Output Layout

Each run uses a sanitized Git branch and an epoch-millisecond timestamp:

```text
results/{git_branch}_{timestamp}/
├── {git_branch}_{timestamp}_stats.csv
├── {git_branch}_{timestamp}_solution.csv
└── {git_branch}_{timestamp}_conflicts.csv
```

Stats and solution are always present after a solver returns. Conflicts is
present only for a failed local-repair run; it may contain only its header when
failure was caused by a missing path rather than a geometric conflict.

The writer first creates the complete bundle in a temporary directory and then
renames that directory to its final name. A partially written run is therefore
not exposed under the final prefix.

## Statistics Schema

```text
map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,continue_if_failed
```

- `num_agents` is the exact non-negative value supplied through `-agents`.
- `paths_resolved` counts non-empty final paths whose agent participates in no
  final conflict.
- `sumOfCosts` and `makespan` include every available final path, even when it
  remains conflicted in a failed run.
- `injustice` is the population standard deviation of the final-versus-optimum
  action-cost differences for agents that have both paths. It is `0.0` when no
  agent has both.
- `durationSeconds` covers the solver call.
- `time` covers instance construction/loading and solving, excluding CSV
  serialization.
- `multithreading` is true only for the parallel local-repair solver.
- `num_threads` is the configured parallel worker count and `1` for sequential
  solvers.
- `solver` is exactly one of the three supported CLI names.
- `continue_if_failed` is its effective boolean value after applying the
  default; priority planning records `false`.

## Solution Schema

```text
agent_id,agent_scenario_bucket,start,goal,optimum_path,solution_path,optimum_path_cost,solution_path_cost,success_optimum_path,success_solution_path
```

There is one row per instance agent in instance order. `agent_id` is the unique
experiment-local solver ID. `agent_scenario_bucket` is the first field read from
the Moving AI scenario row and may repeat; manually created agents use `-1`.

Coordinates use `x-y`. Paths use `x-y|x-y|...` and preserve repeated cells for
wait actions. Costs count actions (`path.size() - 1`), so start-equals-goal has
cost `0`. A missing path is an empty CSV field with cost `-1`.

`success_optimum_path` means the unconstrained A* path exists.
`success_solution_path` means the final path exists and its agent appears in no
final vertex or edge conflict. A present but conflicted path is retained for
diagnosis with `success_solution_path=false`.

## Conflict Schema

```text
cell_1,cell_2,timestep,conflict_type,agents
```

Vertex rows repeat the conflicted cell in both cell fields and use `vertex`.
Edge rows use canonical coordinate-ordered endpoints and `edge`. `agents`
contains all involved experiment-local IDs, sorted and joined with `|`.
Records are deterministically ordered by timestep, type, cells, and IDs.

Collision normalization uses stay-at-target semantics: after a finite path
ends, its agent remains at its final cell indefinitely.
