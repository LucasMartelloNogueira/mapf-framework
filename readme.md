# MAPF Framework

Framework em C++20 para experimentos de Multi-Agent Path Finding (MAPF).

## Build e execução normal

```bash
cmake --preset normal
cmake --build --preset normal --parallel
ctest --preset normal
```

A configuração normal usa `Release` e gera os executáveis em `build/normal`.
Execute experimentos com `scripts/run_experiment.sh normal -- <argumentos>`.

## Build e execução para profiling

```bash
cmake --preset profile
cmake --build --preset profile --parallel
scripts/run_experiment.sh profile -- <argumentos>
```

A configuração de profiling usa `RelWithDebInfo`, símbolos e frame pointers e
fica isolada em `build/profile`. Para coletar as métricas do `perf`, os
relatórios textuais, as pilhas e o FlameGraph SVG em um só comando, use:

```bash
scripts/profile_experiment.sh -- <argumentos>
```

Instalação das ferramentas, exemplos completos, descrição dos artefatos e
solução de problemas estão em
[`docs/perf_flamegraph_profiling.md`](docs/perf_flamegraph_profiling.md).

## CLI de experimentos

Formato geral:

```text
scripts/run_experiment.sh <normal|profile> -- -map <mapa> -scen <cenario> -solver <solver> -agents <n> [-threads <t>] [-continue_if_failed <true|false>]
```

Priority planning:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver PriorityPlanningSolver \
  -agents 3
```

Reparo local com caminhos iniciais calculados em paralelo:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 \
  -threads 8 \
  -continue_if_failed true
```

Reparo local com caminhos iniciais calculados sequencialmente:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairIterativeSolver \
  -agents 100 \
  -continue_if_failed false
```

`-threads` é obrigatório somente para `LocalPathRepairParallelSolver`.
`-continue_if_failed` é aceito somente pelos dois solvers de reparo local e
assume `false` quando omitido.

## Resultados

Cada experimento cria uma pasta própria:

```text
results/{git_branch}_{timestamp}/
├── {git_branch}_{timestamp}_stats.csv
├── {git_branch}_{timestamp}_solution.csv
└── {git_branch}_{timestamp}_conflicts.csv
```

O arquivo de conflitos existe somente quando um solver de reparo local termina
sem sucesso. Resultados parciais continuam disponíveis nos CSVs de estatísticas
e solução.

Os schemas, códigos de saída, convenções de custo e comportamento em falhas
estão detalhados em [`docs/experiment_cli_and_results.md`](docs/experiment_cli_and_results.md).

## Executáveis de exemplo

Os experimentos anteriores continuam disponíveis e usam o mesmo formato de
artefatos:

```bash
./build/normal/manual_experiment
./build/normal/benchmark_experiment
```
