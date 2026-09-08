# MAPF Framework

Framework em C++20 para experimentos de Multi-Agent Path Finding (MAPF).

## Compilação e testes

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

## CLI de experimentos

Formato geral:

```text
./build/mapf_app -map <mapa> -scen <cenario> -solver <solver> -agents <n> [-threads <t>] [-continue_if_failed <true|false>]
```

Priority planning:

```bash
./build/mapf_app \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver PriorityPlanningSolver \
  -agents 3
```

Reparo local com caminhos iniciais calculados em paralelo:

```bash
./build/mapf_app \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 \
  -threads 8 \
  -continue_if_failed true
```

Reparo local com caminhos iniciais calculados sequencialmente:

```bash
./build/mapf_app \
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
./build/manual_experiment
./build/benchmark_experiment
```
