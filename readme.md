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
scripts/run_experiment.sh <normal|profile> -- -map <mapa> -scen <cenario> -solver <solver> -agents <n> [-threads <t>] [-localRepairStrategy <RESOLVE_BY_AGENT|RESOLVE_BY_TIME>] [-maxNewConflicts <n>] [-outputFolder <pasta>]
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
| `-maxNewConflicts` | Quantidade máxima de novos conflitos que um caminho gerado pelo reparo local pode introduzir e ainda ser aceito. | Inteiro maior ou igual a `0`. Opcional, com padrão `0`. Aceito somente por `LocalPathRepairParallelSolver` e `LocalPathRepairIterativeSolver`. |
| `-outputFolder` | Pasta, dentro de `results/`, na qual será criada a pasta própria da execução. | Opcional e aceito por todos os solvers. Deve ser um caminho relativo não vazio, sem `..`; quando omitido, a pasta da execução é criada diretamente em `results/`. |

Nos solvers de reparo local, cada reparo combina o prefixo preservado, uma ponte
SIPP e o restante do caminho antigo. Os conflitos do sufixo são atualizados para
as próximas iterações. Um caminho candidato é aceito quando introduz no máximo
`maxNewConflicts` conflitos novos; com o padrão `0`, somente candidatos sem novos
conflitos são aceitos. Se nenhuma ponte utilizável for encontrada, o solver tenta
SIPP completo com permanência no destino. Se essa busca também falhar, retorna o
último estado confirmado. Não há opção de continuar após uma falha.

Durante o reparo, os solvers locais mantêm um conjunto das configurações já
confirmadas usando hashes de 128 bits. O hash de cada caminho diferencia célula,
instante, repetições e tamanho; sua contribuição também inclui o ID do agente.
Ao testar um novo caminho, o hash global é atualizado retirando a contribuição
antiga daquele agente e inserindo a nova. Se o resultado já estiver no conjunto,
o candidato é rejeitado e `sameConfigCounter` é incrementado, evitando retornar
a uma configuração anterior sem recalcular os hashes dos outros agentes.

O `FullPathRepairIterativeSolver` também tenta todos os caminhos iniciais com
A*. Para cada conflito, retira as reservas de todos os participantes e recalcula
seus caminhos completos com SIPP em ordem crescente de ID, inserindo cada novo
caminho antes da próxima busca. Se algum participante falhar, descarta as
alterações daquele grupo e retorna o último estado confirmado. Usa uma única
construção inicial da tabela de reservas e atualizações incrementais durante o
reparo. Das opções específicas dos solvers, aceita somente os quatro argumentos
obrigatórios; `-threads`, `-localRepairStrategy` e `-maxNewConflicts` são rejeitadas
mesmo quando recebem valores iguais aos padrões dos outros solvers. A opção geral
`-outputFolder` é aceita normalmente.

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
  -agents 3 \
  -outputFolder teste1
```

Reparo local com caminhos iniciais calculados em paralelo:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 \
  -threads 8 \
  -maxNewConflicts 0
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

## Bateria comparativa de experimentos

[`scripts/experiments.sh`](scripts/experiments.sh) executa as 216 combinações da
matriz abaixo, usando o cenário `random-1` correspondente a cada mapa:

| Mapa | Quantidades de agentes |
| --- | --- |
| `den520d` | 500, 700, 900 |
| `empty-8-8` | 8, 16, 32 |
| `empty-32-32` | 300, 400, 500 |
| `random-32-32-20` | 50, 100, 150 |
| `warehouse-10-20-10-2-1` | 150, 250, 350 |
| `ost003d` | 100, 300, 500 |

Para cada quantidade de agentes, são executados os quatro solvers:

- `PriorityPlanningSolver` e `FullPathRepairIterativeSolver`: uma execução cada.
- `LocalPathRepairIterativeSolver`: duas execuções, uma por estratégia local.
- `LocalPathRepairParallelSolver`: oito execuções, combinando 2, 4, 8 e 16 threads
  com `RESOLVE_BY_AGENT` e `RESOLVE_BY_TIME`.

Isso resulta em 36 execuções por mapa. As flags opcionais são enviadas somente
aos solvers que as aceitam. As execuções são sequenciais, com progresso e código
de resultado no terminal; uma falha não impede os próximos casos. Ao final, o
script retorna `2` se houve erro de configuração/execução, `1` se alguma chamada
retornou `1` (sem solução ou falha na gravação), ou `0` se todas tiveram sucesso.

```bash
# Conferir os comandos sem iniciar os solvers; não exige compilação.
scripts/experiments.sh --dry-run

# Executar usando build/normal/mapf_app, compilado previamente.
scripts/experiments.sh

# Opcional: usar build/profile/mapf_app, compilado previamente.
scripts/experiments.sh profile
```

O script avisa sobre cenários com agentes insuficientes antes de iniciar a bateria.

## Resultados

Cada experimento cria uma pasta própria. Sem `-outputFolder`, ela fica diretamente
em `results/`:

```text
results/{git_branch}_{timestamp}/
├── {git_branch}_{timestamp}_stats.csv
├── {git_branch}_{timestamp}_solution.csv
└── {git_branch}_{timestamp}_conflicts.csv
```

Com `-outputFolder teste1`, a mesma pasta fica agrupada um nível abaixo:

```text
results/teste1/{git_branch}_{timestamp}/
├── {git_branch}_{timestamp}_stats.csv
├── {git_branch}_{timestamp}_solution.csv
└── {git_branch}_{timestamp}_conflicts.csv
```

Os arquivos `_stats.csv` e `_solution.csv` são gerados sempre que o solver retorna
e o conjunto de artefatos pode ser gravado. O `_conflicts.csv` é gerado somente
quando um solver de reparo local ou o `FullPathRepairIterativeSolver` termina sem
sucesso. Nesse caso, ele pode conter apenas o cabeçalho se a falha tiver sido
causada por um caminho inexistente, sem um conflito geométrico para registrar.
Resultados parciais permanecem disponíveis nos outros dois arquivos.

### Convenções dos arquivos

- Posições e células usam o formato `x-y`, por exemplo, `12-7`.
- Caminhos usam o formato `x-y|x-y|...`. Células consecutivas repetidas
  representam uma ação de espera.
- O custo de um caminho é o número de ações, isto é, `quantidade_de_células - 1`.
  Portanto, um caminho no qual origem e destino são iguais tem custo `0`.
- Um caminho inexistente é representado por um campo vazio e tem custo `-1`.
- Valores booleanos são serializados como `true` ou `false`.
- Tempos em segundos são escritos com seis casas decimais.

### `_stats.csv`: dados gerais do experimento

Esse arquivo possui uma linha com os parâmetros usados, informações gerais da
instância e métricas globais do resultado. Ele permite comparar execuções sem
examinar os caminhos individuais.

```text
map,instance_name,num_agents,success,paths_resolved,sumOfCosts,makespan,injustice,durationSeconds,time,multithreading,num_threads,solver,local_repair_strategy,maxNewConflicts,sameConfigCounter,num_inital_conflicts,num_resolved_conflicts,num_unresolved_conflicts
```

| Coluna | Significado |
| --- | --- |
| `map` | Nome do arquivo de mapa usado no experimento, incluindo sua extensão. |
| `instance_name` | Nome do arquivo de cenário que originou a instância, incluindo sua extensão. |
| `num_agents` | Quantidade de agentes solicitada por `-agents` e presente na instância. |
| `success` | Resultado global: `true` quando o solver concluiu com um caminho final para todos os agentes e nenhum conflito restante; `false` quando faltou algum caminho, restou algum conflito ou o processo de resolução falhou. |
| `paths_resolved` | Quantidade de agentes que possuem caminho final não vazio e não participam de nenhum conflito final. Pode ser menor que `num_agents` em resultados parciais. |
| `sumOfCosts` | Soma dos custos de todos os caminhos finais disponíveis. Caminhos vazios não contribuem; caminhos existentes ainda contribuem quando permanecem conflitantes em uma execução sem sucesso. |
| `makespan` | Maior custo entre os caminhos finais disponíveis, ou `0` quando nenhum caminho final existe. Também considera caminhos conflitantes preservados para diagnóstico. |
| `injustice` | Desvio-padrão populacional dos acréscimos de custo `solution_path_cost - optimum_path_cost`, considerando somente agentes que possuem os dois caminhos. Mede quão desigualmente o custo adicional foi distribuído; vale `0.0` quando nenhum agente possui ambos os caminhos ou quando todos tiveram o mesmo acréscimo. |
| `durationSeconds` | Tempo, em segundos, gasto dentro da chamada do solver, incluindo a construção dos caminhos iniciais realizada por ele. |
| `time` | Tempo total, em segundos, desde o início do carregamento/construção da instância até o retorno do solver. Não inclui a serialização dos CSVs. |
| `multithreading` | `true` somente para `LocalPathRepairParallelSolver`; `false` para os solvers sequenciais. |
| `num_threads` | Número de workers configurado para `LocalPathRepairParallelSolver`; vale `1` para os solvers sequenciais. |
| `solver` | Solver executado. Valores: `PriorityPlanningSolver`, planejamento sequencial por prioridade; `LocalPathRepairParallelSolver`, caminhos iniciais em paralelo seguidos de reparo local; `LocalPathRepairIterativeSolver`, caminhos iniciais sequenciais seguidos de reparo local; `FullPathRepairIterativeSolver`, replanejamento sequencial dos caminhos completos envolvidos nos conflitos. |
| `local_repair_strategy` | Estratégia dos solvers de reparo local. `RESOLVE_BY_AGENT` seleciona conflitos priorizando a ordem dos agentes; `RESOLVE_BY_TIME` seleciona primeiro o conflito de menor instante; `-` indica que o solver não usa reparo local. |
| `maxNewConflicts` | Valor efetivo de `-maxNewConflicts`. Nos dois solvers de reparo local, limita a quantidade de conflitos novos aceita por caminho candidato; vale `0` por padrão e também para os solvers que não aceitam a flag. |
| `sameConfigCounter` | Quantidade de candidatos rejeitados pelos solvers de reparo local porque seu hash de configuração de 128 bits já havia sido visitado. Vale `0` para os demais solvers. O hash inclui a célula, o instante e o tamanho de cada caminho, além da identidade do agente; após um reparo aceito, somente a contribuição do agente alterado é recalculada. |
| `num_inital_conflicts` | Quantidade de eventos de conflito, de vértice ou de aresta, detectados nos caminhos iniciais. O nome da coluna contém `inital` por compatibilidade com o schema atual. |
| `num_resolved_conflicts` | Quantidade de conflitos iniciais que deixaram de permanecer no resultado, calculada como `max(num_inital_conflicts - num_unresolved_conflicts, 0)`. |
| `num_unresolved_conflicts` | Quantidade de eventos de conflito ainda presentes nos caminhos finais. São os mesmos eventos detalhados no `_conflicts.csv`, quando esse arquivo é gerado. |

### `_solution.csv`: resultado por agente

Esse arquivo representa a saída da instância MAPF. Há uma linha por agente, na
ordem da instância, com origem, destino, caminho individual inicial, caminho
final e indicadores que permitem interpretar soluções completas ou parciais.

```text
agent_id,agent_scenario_bucket,start,goal,optimum_path_cost,solution_path_cost,success_optimum_path,success_solution_path,agentAnchorsIndex,optimum_path,solution_path
```

| Coluna | Significado |
| --- | --- |
| `agent_id` | Identificador único do agente dentro do experimento. Os IDs são normalizados de `0` a `num_agents - 1` e também correspondem à posição do agente na instância. |
| `agent_scenario_bucket` | Bucket informado no primeiro campo da linha do cenário Moving AI. Ele identifica o grupo do caso no arquivo de cenário, pode se repetir entre agentes e vale `-1` para agentes criados manualmente. |
| `start` | Posição inicial do agente no formato `x-y`. |
| `goal` | Posição de destino do agente no formato `x-y`. |
| `optimum_path_cost` | Custo do caminho individual inicial encontrado por A* sem considerar os demais agentes. Vale `-1` quando esse caminho não existe. |
| `solution_path_cost` | Custo do caminho final retornado para o agente. Vale `-1` quando nenhum caminho final existe. Um caminho presente pode ter custo válido mesmo se ainda estiver em conflito. |
| `success_optimum_path` | `true` quando o A* encontrou o caminho individual inicial; `false` quando `optimum_path` está vazio. |
| `success_solution_path` | `true` quando o caminho final existe e o agente não participa de nenhum conflito final; `false` quando o caminho está ausente ou ainda conflita. Esse valor é individual e pode ser `true` mesmo quando `success` no `_stats.csv` é `false` por causa de outro agente. |
| `agentAnchorsIndex` | Resultado da tentativa de reparo mais recente nos dois solvers locais. Valores não negativos são o ordinal da âncora local aceita: `0` é a primeira âncora tentada, `1` é a segunda e assim por diante; o número não é o timestep da âncora. `-1` significa que o agente não teve seu caminho alterado pelo reparo; `-2` significa que as tentativas locais não produziram um caminho aceito e o replanejamento completo foi usado com sucesso; `-3` significa que tanto o reparo por âncoras quanto o replanejamento completo falharam. Os demais solvers registram `-`, pois não produzem esse metadado. |
| `optimum_path` | Caminho individual inicial calculado por A*, no formato `x-y\|x-y\|...`, sem considerar conflitos com outros agentes. Fica vazio quando nenhum caminho foi encontrado. |
| `solution_path` | Último caminho confirmado para o agente, no mesmo formato. Em uma execução sem sucesso, pode ficar vazio ou conter um caminho preservado que ainda participa de conflitos. |

### `_conflicts.csv`: conflitos não resolvidos

Esse arquivo serve para diagnosticar os conflitos que permanecem nos caminhos
finais de uma execução sem sucesso. Cada linha representa um evento de conflito,
e não um par separado para cada combinação de agentes. Os IDs de todos os
participantes do mesmo evento são agrupados na coluna `agents`.

```text
cell_1,cell_2,timestep,conflict_type,agents
```

| Coluna | Significado |
| --- | --- |
| `cell_1` | Primeira célula do conflito, no formato `x-y`. Em conflitos de vértice, é a própria célula compartilhada. Em conflitos de aresta, é a primeira extremidade da aresta na ordenação canônica por coordenadas. |
| `cell_2` | Segunda célula do conflito. Repete `cell_1` em conflitos de vértice; em conflitos de aresta, contém a segunda extremidade da aresta na ordenação canônica. As duas colunas não indicam a direção de movimento dos agentes. |
| `timestep` | Instante discreto do conflito. Para `vertex`, é o instante em que os agentes ocupam a mesma célula; para `edge`, é o instante de chegada do movimento em sentidos opostos, correspondente à transição de `timestep - 1` para `timestep`. |
| `conflict_type` | Tipo enumerado do conflito: `vertex` quando dois ou mais agentes ocupam a mesma célula no mesmo instante; `edge` quando agentes atravessam a mesma aresta em sentidos opostos no mesmo intervalo. |
| `agents` | IDs de todos os agentes envolvidos no evento, ordenados de forma crescente e separados por `\|`, por exemplo, `0\|3\|7`. |

A detecção considera que o agente permanece indefinidamente em sua célula de
destino depois do fim do caminho. Portanto, uma visita posterior de outro agente
a esse destino também pode gerar um conflito de vértice. Os registros são
ordenados deterministicamente por `timestep`, tipo e células.

Detalhes adicionais sobre os códigos de saída e o comportamento em falhas estão
em [`docs/experiment_cli_and_results.md`](docs/experiment_cli_and_results.md).

## Executáveis de exemplo

Os experimentos anteriores continuam disponíveis e usam o mesmo formato de
artefatos:

```bash
./build/normal/manual_experiment
./build/normal/benchmark_experiment
```
