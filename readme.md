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
scripts/run_experiment.sh <normal|profile> -- -map <mapa> -scen <cenario> -solver <solver> -agents <n> [-threads <t>] [-continue_if_failed <true|false>] [-localRepairStrategy <RESOLVE_BY_AGENT|RESOLVE_BY_TIME>]
```

### Argumentos da CLI

Os argumentos são informados em pares `-argumento valor`, em qualquer ordem,
sem repetir flags. Os nomes dos argumentos e seus valores textuais devem
respeitar maiúsculas e minúsculas. No formato acima, os colchetes indicam
argumentos opcionais, conforme as regras de cada solver.

| Argumento | Significado | Valores aceitos e regras de uso |
| --- | --- | --- |
| `-map` | Mapa usado no experimento. | Obrigatório. Caminho de um arquivo `.map` em `benchmarks/maps`, por exemplo, `benchmarks/maps/empty-8-8.map`. |
| `-scen` | Arquivo de cenário que define as posições iniciais e os destinos dos agentes. | Obrigatório. Caminho de um arquivo `.scen` em `benchmarks/scenarios/{map_name}/{scen_type}`, em que `map_name` é o nome do mapa sem a extensão e `scen_type` é o tipo de cenário, como `random`. O cenário deve corresponder ao mapa de `-map`. |
| `-solver` | Estratégia usada para resolver a instância. | Obrigatório. Aceita `PriorityPlanningSolver` (planejamento por prioridade), `LocalPathRepairParallelSolver` (reparo local com cálculo paralelo dos caminhos iniciais) ou `LocalPathRepairIterativeSolver` (reparo local com cálculo sequencial dos caminhos iniciais). |
| `-agents` | Número de agentes usados no experimento. Para `k` agentes, lê os primeiros `k` agentes do arquivo indicado em `-scen`. | Obrigatório. Inteiro maior ou igual a `0`; o cenário deve conter pelo menos essa quantidade de agentes. |
| `-threads` | Número de threads usadas para encontrar os caminhos iniciais em `LocalPathRepairParallelSolver`. | Inteiro maior que `0`. Obrigatório para `LocalPathRepairParallelSolver` e não aceito pelos outros solvers. |
| `-continue_if_failed` | Define se o reparo local continua após não conseguir encontrar um caminho sem conflitos para um agente. Com `false`, o experimento para na primeira falha de reparo; com `true`, continua tentando resolver os conflitos dos demais agentes. | `true` ou `false`. Opcional, com padrão `false`. Aceito somente por `LocalPathRepairParallelSolver` e `LocalPathRepairIterativeSolver`. |
| `-localRepairStrategy` | Estratégia de resolução de conflitos no reparo local: `RESOLVE_BY_AGENT` prioriza a ordem dos agentes; `RESOLVE_BY_TIME` prevê priorizar os conflitos pelo instante em que ocorrem (`RESOLVE_BY_TIME` ainda não foi implementado). | `RESOLVE_BY_AGENT` ou `RESOLVE_BY_TIME`. Opcional, com padrão `RESOLVE_BY_AGENT`. Aceito somente por `LocalPathRepairParallelSolver` e `LocalPathRepairIterativeSolver`. |

Nos solvers de reparo local, a busca dos caminhos iniciais é tentada para todos
os agentes antes da etapa de reparo. Se algum caminho inicial não existir, o
experimento termina sem sucesso, independentemente de `-continue_if_failed`.
Continuar após uma falha de reparo permite obter resultados parciais, mas não
garante uma solução completa.

O script `scripts/run_experiment.sh` também recebe:

- `normal` ou `profile`: primeiro argumento, que seleciona o executável em
  `build/normal` ou `build/profile`, respectivamente. A compilação correspondente
  deve ter sido feita previamente.
- `--`: separador entre o modo de execução e os argumentos do experimento.
- `-h` ou `--help`: quando usado como primeiro argumento, mostra a ajuda do
  script, por exemplo, `scripts/run_experiment.sh --help`.

### Exemplos

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
