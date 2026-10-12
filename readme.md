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
scripts/run_experiment.sh <normal|profile> -- -map <mapa> -scen <cenario> -solver <solver> -agents <n> [-threads <t>] [-localRepairStrategy <RESOLVE_BY_AGENT|RESOLVE_BY_TIME>]
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
| `-solver` | Estratégia usada para resolver a instância. | Obrigatório. Aceita `PriorityPlanningSolver` (planejamento por prioridade), `LocalPathRepairParallelSolver` (reparo local com cálculo paralelo dos caminhos iniciais), `LocalPathRepairIterativeSolver` (reparo local com cálculo sequencial dos caminhos iniciais) ou `FullPathRepairIterativeSolver` (reparo sequencial dos caminhos completos de todos os participantes do conflito). |
| `-agents` | Número de agentes usados no experimento. Para `k` agentes, lê os primeiros `k` agentes do arquivo indicado em `-scen`. | Obrigatório. Inteiro maior ou igual a `0`; o cenário deve conter pelo menos essa quantidade de agentes. |
| `-threads` | Número de threads usadas para encontrar os caminhos iniciais em `LocalPathRepairParallelSolver`. | Inteiro maior que `0`. Obrigatório para `LocalPathRepairParallelSolver` e não aceito pelos outros solvers. |
| `-localRepairStrategy` | Estratégia de resolução de conflitos no reparo local: `RESOLVE_BY_AGENT` prioriza a ordem dos agentes; `RESOLVE_BY_TIME` prioriza os conflitos pelo instante em que ocorrem. | `RESOLVE_BY_AGENT` ou `RESOLVE_BY_TIME`. Opcional, com padrão `RESOLVE_BY_AGENT`. Aceito somente por `LocalPathRepairParallelSolver` e `LocalPathRepairIterativeSolver`. |

Nos solvers de reparo local, cada reparo combina o prefixo preservado, uma ponte
SIPP e o restante do caminho antigo. Os conflitos do sufixo são atualizados para
as próximas iterações. Se nenhuma ponte utilizável for encontrada, o solver tenta
SIPP completo com permanência no destino. Se essa busca também falhar, retorna
o último estado confirmado. Não há opção de continuar após uma falha.

O `FullPathRepairIterativeSolver` também tenta todos os caminhos iniciais com
A*. Para cada conflito, retira as reservas de todos os participantes e recalcula
seus caminhos completos com SIPP em ordem crescente de ID, inserindo cada novo
caminho antes da próxima busca. Se algum participante falhar, descarta as
alterações daquele grupo e retorna o último estado confirmado. Usa uma única
construção inicial da tabela de reservas e atualizações incrementais durante o
reparo. Aceita somente os quatro argumentos obrigatórios; as duas flags opcionais
são rejeitadas mesmo quando recebem valores iguais aos padrões dos outros solvers.

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
  -threads 8
```

Reparo local com caminhos iniciais calculados sequencialmente:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairIterativeSolver \
  -agents 100
```

Reparo de caminhos completos:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver FullPathRepairIterativeSolver \
  -agents 3
```

O algoritmo, as regras de prioridade e o comportamento em falhas estão em
[`docs/full_path_repair_iterative_solver.md`](docs/full_path_repair_iterative_solver.md).

## Resultados

Cada experimento cria uma pasta própria:

```text
results/{git_branch}_{timestamp}/
├── {git_branch}_{timestamp}_stats.csv
├── {git_branch}_{timestamp}_solution.csv
└── {git_branch}_{timestamp}_conflicts.csv
```

O arquivo de conflitos existe quando um solver de reparo local ou o
`FullPathRepairIterativeSolver` termina sem sucesso. Pode conter apenas o cabeçalho
quando a falha decorre de um caminho ausente. Resultados parciais continuam
disponíveis nos CSVs de estatísticas e solução. O reparo completo registra
`local_repair_strategy=-` e uma thread.

Os schemas, códigos de saída, convenções de custo e comportamento em falhas
estão detalhados em [`docs/experiment_cli_and_results.md`](docs/experiment_cli_and_results.md).

## Executáveis de exemplo

Os experimentos anteriores continuam disponíveis e usam o mesmo formato de
artefatos:

```bash
./build/normal/manual_experiment
./build/normal/benchmark_experiment
```
