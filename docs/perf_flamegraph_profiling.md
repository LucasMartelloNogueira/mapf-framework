# Profiling de experimentos com perf e FlameGraph

Este guia descreve o fluxo completo para compilar um experimento MAPF com
símbolos, medir seu uso de CPU com o Linux `perf` e gerar um flame graph em SVG.
Os comandos devem ser executados na raiz do repositório.

## Configurações disponíveis

O projeto mantém configurações independentes para que uma recompilação de
profiling não altere o executável usado nos experimentos normais:

| Preset | Tipo CMake | Diretório | Finalidade |
| --- | --- | --- | --- |
| `normal` | `Release` | `build/normal` | experimentos comuns e testes |
| `profile` | `RelWithDebInfo` | `build/profile` | profiling com otimizações e símbolos |

Embora a segunda configuração seja informalmente chamada de build de debug
para profiling, ela usa `RelWithDebInfo`, não `Debug`. Uma build `Debug` usa
normalmente `-O0` e pode tornar operações artificialmente caras. A configuração
`RelWithDebInfo` mantém as otimizações de uma execução representativa e adiciona
informações de depuração (`-g`). Para compiladores GNU e Clang, o projeto também
adiciona `-fno-omit-frame-pointer`, necessário para pilhas mais confiáveis com
`perf --call-graph fp`.

## 1. Instalar as ferramentas

No Ubuntu, instale o `perf` correspondente ao kernel em execução, Git e Perl:

```bash
sudo apt update
sudo apt install \
  linux-tools-common \
  linux-tools-generic \
  linux-tools-$(uname -r) \
  git \
  perl
```

Confirme que o executável está disponível:

```bash
perf version
```

Clone os scripts oficiais do FlameGraph de Brendan Gregg. A pasta é ignorada
pelo Git deste projeto:

```bash
mkdir -p tools
git clone https://github.com/brendangregg/FlameGraph.git tools/FlameGraph
```

O fluxo usa estes dois scripts:

```text
tools/FlameGraph/stackcollapse-perf.pl
tools/FlameGraph/flamegraph.pl
```

## 2. Build e execução normal

Configure e compile o executável otimizado de uso cotidiano:

```bash
cmake --preset normal
cmake --build --preset normal --parallel
```

Execute um experimento por meio do runner, informando `normal`:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver PriorityPlanningSolver \
  -agents 3
```

O comando equivale a chamar diretamente:

```bash
./build/normal/mapf_app \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver PriorityPlanningSolver \
  -agents 3
```

Os testes pertencem à configuração normal:

```bash
ctest --preset normal
```

## 3. Build e execução de profiling

Configure e compile a build com otimizações, símbolos e frame pointers:

```bash
cmake --preset profile
cmake --build --preset profile --parallel
```

Antes de coletar o perfil, faça uma execução simples para confirmar os
argumentos e os arquivos de entrada:

```bash
scripts/run_experiment.sh profile -- \
  -map benchmarks/maps/empty-8-8.map \
  -scen benchmarks/scenarios/empty-8-8/random/empty-8-8-random-1.scen \
  -solver PriorityPlanningSolver \
  -agents 3
```

É possível conferir as opções usadas pelo compilador em
`build/profile/compile_commands.json`. Com GCC, por exemplo, as entradas devem
conter opções equivalentes a `-O2 -g -DNDEBUG -fno-omit-frame-pointer`.

## 4. Gerar todos os artefatos automaticamente

Escolha uma instância representativa que dure alguns segundos e mantenha mapa,
cenário, agentes, solver, número de threads e demais flags constantes entre as
comparações. O exemplo a seguir usa o solver paralelo:

```bash
scripts/profile_experiment.sh -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 \
  -threads 8 \
  -continue_if_failed true
```

O script faz duas execuções por padrão: uma para `perf stat` e outra para
`perf record`. Cada execução de `mapf_app` também cria seu próprio bundle CSV
em `results/`.

Os artefatos de profiling ficam em um diretório único como:

```text
profiling/20260909T234500Z-12345/
├── metadata.txt
├── perf-stat.txt
├── perf.data
├── perf-report.txt
├── perf-self-report.txt
├── perf.stacks
├── perf.folded
└── flamegraph.svg
```

Ao terminar, o script imprime o caminho exato. Para escolher o diretório e
repetir a coleta de métricas cinco vezes:

```bash
PROFILE_OUTPUT_DIR=profiling/den520d-100-agents \
PERF_STAT_REPETITIONS=5 \
scripts/profile_experiment.sh -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 \
  -threads 8 \
  -continue_if_failed true
```

O diretório fornecido por `PROFILE_OUTPUT_DIR` não pode existir, para impedir a
sobrescrita acidental de uma coleta. `PERF_FREQUENCY` altera a frequência de
amostragem, cujo padrão é `199` Hz:

```bash
PERF_FREQUENCY=99 scripts/profile_experiment.sh -- <argumentos-do-mapf_app>
```

Se o FlameGraph tiver sido clonado em outro local, indique-o com
`FLAMEGRAPH_DIR=/caminho/para/FlameGraph`.

Uma frequência maior coleta mais amostras, mas aumenta o overhead e o tamanho
de `perf.data`. Se o FlameGraph foi instalado em outro local, informe-o com
`FLAMEGRAPH_DIR=/caminho/para/FlameGraph`.

## 5. Conteúdo dos artefatos

| Arquivo | Conteúdo |
| --- | --- |
| `metadata.txt` | revisão Git, comando, build, versão do `perf`, frequência e códigos de saída |
| `perf-stat.txt` | contadores gerais e tempo da execução |
| `perf.data` | amostras binárias originais, incluindo pilhas |
| `perf-report.txt` | relatório textual com custo acumulado nos chamadores |
| `perf-self-report.txt` | relatório textual focado no custo próprio das funções |
| `perf.stacks` | pilhas convertidas em texto por `perf script` |
| `perf.folded` | pilhas idênticas agrupadas para o FlameGraph |
| `flamegraph.svg` | flame graph interativo final |

Em `perf stat`, observe principalmente `task-clock`, CPUs utilizadas,
`instructions`, `cycles`, instruções por ciclo, cache misses, branch misses e
context switches. A disponibilidade dos contadores depende do processador, do
kernel e do ambiente de virtualização.

## 6. Inspecionar os resultados

Defina abaixo o diretório impresso pelo script:

```bash
PROFILE_DIR=profiling/20260909T234500Z-12345
```

Abra o relatório interativo:

```bash
perf report -i "$PROFILE_DIR/perf.data"
```

Use as setas e `Enter` para navegar pelas pilhas e `q` para sair. No relatório:

- `Self` é o custo amostrado diretamente na função;
- `Children` é o custo acumulado das funções chamadas por ela;
- uma função com `Self` baixo e `Children` alto inicia trabalho caro em outras
  funções.

Depois de identificar uma função cara, relacione amostras, assembly e, quando
disponível, linhas do código-fonte:

```bash
perf annotate -i "$PROFILE_DIR/perf.data"
```

Abra o flame graph:

```bash
xdg-open "$PROFILE_DIR/flamegraph.svg"
```

Cada retângulo representa uma função. A largura corresponde à quantidade de
CPU atribuída à função e à pilha associada; a altura representa profundidade de
chamadas, não duração; e o eixo horizontal não é uma linha do tempo. Clique em
um bloco para ampliar e use a busca do SVG para localizar termos como `SIPP`,
`AStar`, `Conflict`, `priority_queue`, `malloc` ou `operator new`.

## 7. Processo manual equivalente

O helper automatiza a sequência abaixo. Executá-la manualmente é útil para
variar eventos e opções do `perf`:

```bash
mkdir -p profiling/manual

perf stat \
  -o profiling/manual/perf-stat.txt \
  -- ./build/profile/mapf_app <argumentos-do-mapf_app>

perf record \
  -F 199 \
  -g \
  --call-graph fp \
  -o profiling/manual/perf.data \
  -- ./build/profile/mapf_app <argumentos-do-mapf_app>

perf report \
  -i profiling/manual/perf.data \
  --stdio \
  --sort=overhead,symbol,dso \
  > profiling/manual/perf-report.txt

perf report \
  -i profiling/manual/perf.data \
  --stdio \
  --no-children \
  --sort=overhead,symbol,dso \
  > profiling/manual/perf-self-report.txt

perf script \
  -i profiling/manual/perf.data \
  > profiling/manual/perf.stacks

tools/FlameGraph/stackcollapse-perf.pl \
  profiling/manual/perf.stacks \
  > profiling/manual/perf.folded

tools/FlameGraph/flamegraph.pl \
  --title "MAPF CPU Flame Graph" \
  --countname samples \
  profiling/manual/perf.folded \
  > profiling/manual/flamegraph.svg
```

Para coletar eventos explícitos em um experimento multithread:

```bash
perf stat \
  -e task-clock,cycles,instructions,cache-references,cache-misses,branches,branch-misses,context-switches,cpu-migrations \
  -- ./build/profile/mapf_app <argumentos-do-mapf_app>
```

O `perf record` acompanha as threads criadas pelo processo. Um flame graph de
CPU mostra principalmente onde as threads consumiram CPU; ele não representa
diretamente todo o tempo em que ficaram bloqueadas ou dormindo.

## 8. Comparar uma otimização

Use exatamente o mesmo experimento antes e depois da mudança, de preferência
em uma máquina ociosa e com o mesmo número de threads:

```bash
PROFILE_OUTPUT_DIR=profiling/before \
PERF_STAT_REPETITIONS=5 \
scripts/profile_experiment.sh -- <argumentos-do-mapf_app>

# Recompile build/profile após aplicar a otimização.

PROFILE_OUTPUT_DIR=profiling/after \
PERF_STAT_REPETITIONS=5 \
scripts/profile_experiment.sh -- <argumentos-do-mapf_app>

perf diff profiling/before/perf.data profiling/after/perf.data
```

Compare também o tempo total e as métricas algorítmicas dos CSVs, como custo,
makespan e quantidade de caminhos resolvidos. Uma função passar a ocupar uma
porcentagem maior não prova que ficou mais lenta: outra região pode ter ficado
mais rápida.

## 9. Problemas comuns

### `No permission to enable ... event`

Confira a política atual:

```bash
cat /proc/sys/kernel/perf_event_paranoid
```

Em uma máquina própria, uma liberação temporária comum para perfilar o próprio
processo é:

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

A mudança reduz uma proteção do sistema e normalmente dura até a
reinicialização. Em máquinas compartilhadas, siga a política do administrador.
Executar apenas a coleta com `sudo` é outra possibilidade, mas pode mudar o
usuário, o ambiente e a propriedade dos artefatos.

### Relatório com `[unknown]` ou sem linhas do fonte

Confirme o tipo da build e a presença do frame pointer:

```bash
grep '^CMAKE_BUILD_TYPE:' build/profile/CMakeCache.txt
grep -- '-fno-omit-frame-pointer' build/profile/compile_commands.json
```

Não recompile nem substitua `build/profile/mapf_app` entre `perf record` e a
análise de `perf.data`. Bibliotecas do sistema também precisam de seus próprios
símbolos para que funções internas sejam resolvidas.

### Pilhas incompletas

Como alternativa à coleta por frame pointer, teste DWARF. Ela tende a gerar
mais overhead e arquivos maiores:

```bash
perf record \
  -F 99 \
  -g \
  --call-graph dwarf \
  -o profiling/perf-dwarf.data \
  -- ./build/profile/mapf_app <argumentos-do-mapf_app>
```

### Flame graph vazio

Confirme que cada estágio contém dados:

```bash
ls -lh \
  "$PROFILE_DIR/perf.data" \
  "$PROFILE_DIR/perf.stacks" \
  "$PROFILE_DIR/perf.folded" \
  "$PROFILE_DIR/flamegraph.svg"
```

Uma execução muito curta pode não produzir amostras suficientes. Verifique
também erros de permissão e resolução de símbolos emitidos pelo `perf`.

## Referências

- [Tutorial do Linux perf](https://perfwiki.github.io/main/tutorial/)
- [FlameGraph, de Brendan Gregg](https://github.com/brendangregg/FlameGraph)
- [Flame Graphs](https://www.brendangregg.com/flamegraphs.html)
