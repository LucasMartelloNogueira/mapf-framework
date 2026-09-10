# Tutorial: profiling de um programa MAPF com `perf` e FlameGraph

Este tutorial mostra como analisar um programa MAPF escrito em C++20 e compilado com CMake no Ubuntu. Ao final, você terá:

- uma tabela interativa das funções que mais consumiram CPU;
- uma versão textual dessa tabela, que pode ser salva em arquivo;
- um flame graph em SVG;
- símbolos de funções, pilhas de chamadas e, quando possível, linhas do código-fonte;
- métricas auxiliares como cache misses, branch misses e context switches.

## 1. Por que usar `RelWithDebInfo`

O profiling deve representar a execução real do algoritmo. Uma build `Debug`, normalmente compilada sem otimizações (`-O0`), pode tornar operações artificialmente caras e produzir conclusões enganosas.

O tipo de build `RelWithDebInfo` combina:

- otimizações de compilação;
- símbolos de depuração (`-g`);
- desempenho próximo ao de uma build de release.

Também usaremos `-fno-omit-frame-pointer`. Essa opção conserva os *frame pointers* e ajuda o `perf` a reconstruir corretamente as pilhas de chamadas.

## 2. Configurar o CMake

No `CMakeLists.txt`, confirme que C++20 está ativado:

```cmake
cmake_minimum_required(VERSION 3.16)
project(mapf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(mapf
    src/main.cpp
    # Adicione aqui os outros arquivos .cpp do projeto.
)

target_include_directories(mapf PRIVATE include)

# Facilita a reconstrução das pilhas de chamadas pelo perf.
target_compile_options(mapf PRIVATE
    $<$<CONFIG:RelWithDebInfo>:-fno-omit-frame-pointer>
)
```

Adapte apenas o nome do executável, os fontes e os diretórios de include à estrutura real do projeto.

Configure e compile a build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel
```

Se o alvo se chamar `mapf`, o executável normalmente ficará em:

```text
build/mapf
```

Confirme que ele funciona antes do profiling:

```bash
./build/mapf <argumentos-do-programa>
```

> Se você usa um gerador CMake de múltiplas configurações, como Ninja Multi-Config, compile com `cmake --build build --config RelWithDebInfo` e ajuste o caminho do executável.

## 3. Instalar o `perf` e obter o FlameGraph

Instale o `perf` correspondente ao kernel em execução e o Git:

```bash
sudo apt update
sudo apt install linux-tools-common linux-tools-generic linux-tools-$(uname -r) git
```

Confirme a instalação:

```bash
perf --version
```

Clone o repositório oficial do FlameGraph de Brendan Gregg. Você pode mantê-lo dentro de uma pasta de ferramentas separada do projeto:

```bash
mkdir -p tools
git clone https://github.com/brendangregg/FlameGraph.git tools/FlameGraph
```

Os scripts usados neste tutorial serão:

```text
tools/FlameGraph/stackcollapse-perf.pl
tools/FlameGraph/flamegraph.pl
```

## 4. Usar um experimento reproduzível

Escolha uma execução representativa, mas não excessivamente curta. Idealmente, ela deve durar pelo menos alguns segundos para que o `perf` colete amostras suficientes.

Em um algoritmo MAPF, mantenha constantes:

- arquivo do mapa;
- arquivo de cenário;
- quantidade de agentes;
- limite de tempo;
- tamanho da vizinhança;
- semente aleatória;
- número de threads;
- demais parâmetros do algoritmo.

Exemplo fictício usado nas próximas seções:

```bash
./build/mapf --map maps/Berlin_1_256.map --scenario scenarios/test.scen --agents 100 --seed 42
```

Substitua esse comando pela interface real do seu programa.

## 5. Obter métricas gerais com `perf stat`

Antes de descobrir quais funções são caras, obtenha uma visão geral:

```bash
perf stat -- ./build/mapf <argumentos-do-programa>
```

Para repetir a mesma execução cinco vezes e obter média e variação:

```bash
perf stat -r 5 -- ./build/mapf <argumentos-do-programa>
```

Algumas métricas importantes:

| Métrica | Interpretação |
| --- | --- |
| `task-clock` | Tempo de CPU efetivamente consumido |
| `CPUs utilized` | Paralelismo médio utilizado durante a execução |
| `instructions` | Quantidade de instruções executadas |
| `cycles` | Ciclos de CPU consumidos |
| `insn per cycle` | Instruções por ciclo; um valor muito baixo pode indicar esperas ou ineficiência microarquitetural |
| `cache-misses` | Falhas de cache; podem indicar acessos ruins à memória |
| `branch-misses` | Erros de predição de desvios |
| `context-switches` | Trocas de contexto; podem crescer com excesso de threads ou sincronização |

Nem todos os eventos estão disponíveis em todo processador ou ambiente virtualizado.

## 6. Gravar o perfil da execução

Execute:

```bash
perf record \
    -F 199 \
    -g \
    --call-graph fp \
    -o perf-mapf.data \
    -- ./build/mapf <argumentos-do-programa>
```

Significado das opções:

| Opção | Função |
| --- | --- |
| `-F 199` | Tenta coletar 199 amostras por segundo |
| `-g` | Registra as pilhas de chamadas |
| `--call-graph fp` | Reconstrói as pilhas usando frame pointers |
| `-o perf-mapf.data` | Define o arquivo de saída |
| `--` | Separa as opções do `perf` do comando do programa |

Uma frequência como 99 ou 199 Hz costuma ser um bom ponto de partida. Frequências maiores geram mais amostras, porém aumentam o overhead e o tamanho do arquivo.

Ao término, o arquivo `perf-mapf.data` conterá as amostras da execução.

## 7. Ver a tabela das funções mais caras

### Tabela interativa

Abra o relatório:

```bash
perf report -i perf-mapf.data
```

Use as setas e `Enter` para navegar pelas pilhas. Pressione `q` para sair.

Os conceitos principais são:

- **Self**: amostras coletadas diretamente na função;
- **Children**: custo acumulado das funções chamadas por ela;
- **Overhead**: porcentagem das amostras atribuída à função, de acordo com o modo do relatório.

Uma função pode ter `Self` baixo e `Children` alto. Nesse caso, ela não executa muito trabalho diretamente, mas inicia uma parte cara da árvore de chamadas.

### Tabela textual com custo acumulado

Para salvar um relatório legível:

```bash
perf report \
    -i perf-mapf.data \
    --stdio \
    --sort=overhead,symbol,dso \
    > perf-mapf-report.txt
```

### Tabela focada no custo próprio

Para evitar que o custo dos filhos seja acumulado nos chamadores:

```bash
perf report \
    -i perf-mapf.data \
    --stdio \
    --no-children \
    --sort=overhead,symbol,dso \
    > perf-mapf-self-report.txt
```

Esse segundo relatório é útil para responder: “em qual função a CPU estava executando diretamente?”. Analise os dois relatórios em conjunto.

### Inspecionar linhas e assembly

Após identificar uma função cara, use:

```bash
perf annotate -i perf-mapf.data
```

O `perf annotate` relaciona as amostras com instruções de assembly e, quando os símbolos permitem, com linhas do código-fonte.

## 8. Gerar o flame graph

Converta as amostras do `perf` para texto:

```bash
perf script -i perf-mapf.data > perf-mapf.stacks
```

Agrupe pilhas iguais:

```bash
tools/FlameGraph/stackcollapse-perf.pl \
    perf-mapf.stacks \
    > perf-mapf.folded
```

Gere o SVG:

```bash
tools/FlameGraph/flamegraph.pl \
    --title "MAPF CPU Flame Graph" \
    --countname samples \
    perf-mapf.folded \
    > mapf-flamegraph.svg
```

Abra no navegador:

```bash
xdg-open mapf-flamegraph.svg
```

O SVG é interativo:

- clique em um bloco para ampliar aquela pilha;
- use a busca para localizar nomes como `SIPP`, `AStar`, `Conflict` ou `std::priority_queue`;
- clique em **Reset Zoom** para retornar à visão completa.

## 9. Como interpretar o flame graph

Cada retângulo representa uma função presente nas pilhas amostradas.

| Característica | Significado |
| --- | --- |
| Largura do bloco | Quantidade de CPU atribuída à função e às chamadas abaixo/acima dela na pilha |
| Altura | Profundidade da pilha de chamadas, não duração |
| Eixo horizontal | Organização das pilhas; não representa uma linha do tempo |
| Cor padrão | Geralmente decorativa; não significa automaticamente “rápido” ou “lento” |

Procure blocos largos. Em um programa MAPF, candidatos comuns incluem:

- expansão de nós no A* ou SIPP;
- inserções e remoções da `std::priority_queue`;
- hashing de estados espaço-temporais;
- construção de safe intervals;
- detecção e atualização de conflitos;
- cópias de caminhos;
- alocações feitas por `operator new` ou `malloc`;
- sincronização e contenção entre threads.

Esses são apenas pontos a observar. O perfil medido deve determinar onde otimizar.

## 10. Execução completa em sequência

Depois que tudo estiver configurado, este é o fluxo normal:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --parallel

perf stat -- ./build/mapf <argumentos-do-programa>

perf record \
    -F 199 \
    -g \
    --call-graph fp \
    -o perf-mapf.data \
    -- ./build/mapf <argumentos-do-programa>

perf report -i perf-mapf.data

perf report \
    -i perf-mapf.data \
    --stdio \
    --no-children \
    --sort=overhead,symbol,dso \
    > perf-mapf-self-report.txt

perf script -i perf-mapf.data > perf-mapf.stacks

tools/FlameGraph/stackcollapse-perf.pl \
    perf-mapf.stacks \
    > perf-mapf.folded

tools/FlameGraph/flamegraph.pl \
    --title "MAPF CPU Flame Graph" \
    --countname samples \
    perf-mapf.folded \
    > mapf-flamegraph.svg

xdg-open mapf-flamegraph.svg
```

## 11. Problemas comuns

### `perf: command not found`

Instale os pacotes associados ao kernel atual:

```bash
sudo apt install linux-tools-common linux-tools-generic linux-tools-$(uname -r)
```

Após uma atualização do kernel, reinicie o computador se os pacotes instalados e o kernel em execução não corresponderem.

### `No permission to enable cycles event`

Veja a restrição atual:

```bash
cat /proc/sys/kernel/perf_event_paranoid
```

Para liberar temporariamente o profiling do próprio processo:

```bash
sudo sysctl kernel.perf_event_paranoid=1
```

Essa alteração normalmente dura até a reinicialização. Ela reduz uma proteção do sistema; em uma máquina compartilhada, confirme a política do ambiente antes de alterá-la. Outra possibilidade é executar apenas o comando de coleta com `sudo`, mas isso pode mudar usuário, ambiente e acesso aos arquivos do programa.

### O relatório mostra endereços ou `[unknown]`

Verifique se:

1. a build usada é `RelWithDebInfo`;
2. o executável contém símbolos de depuração;
3. o executável não foi recompilado depois da coleta;
4. `-fno-omit-frame-pointer` foi aplicado ao alvo correto;
5. as bibliotecas relevantes possuem símbolos disponíveis.

Confira o tipo de build no cache do CMake:

```bash
grep CMAKE_BUILD_TYPE build/CMakeCache.txt
```

### A pilha está incompleta mesmo com frame pointers

Confirme nos comandos de compilação se a opção foi realmente usada:

```bash
cmake --build build --verbose
```

Como alternativa, você pode testar pilhas DWARF:

```bash
perf record \
    -F 99 \
    -g \
    --call-graph dwarf \
    -o perf-mapf-dwarf.data \
    -- ./build/mapf <argumentos-do-programa>
```

DWARF pode reconstruir pilhas sem depender tanto de frame pointers, mas costuma gerar arquivos maiores e mais overhead.

### O flame graph está vazio

Verifique se cada etapa produziu dados:

```bash
ls -lh perf-mapf.data perf-mapf.stacks perf-mapf.folded mapf-flamegraph.svg
```

Também confirme que o programa executou por tempo suficiente e que `perf script` não exibiu erros de símbolos ou permissões.

### Muitos nomes internos da STL aparecem no gráfico

Isso é normal em C++. Comece pela função do seu projeto que chama aquela operação. Por exemplo, uma região larga contendo funções da `std::priority_queue` pode indicar que o algoritmo realiza muitas operações na open list; não significa necessariamente que a implementação da biblioteca esteja errada.

## 12. Profiling de um programa multithread

O `perf record` acompanha, por padrão, as threads criadas pelo processo iniciado. Para analisar o uso global de CPU, observe `CPUs utilized` no `perf stat`.

Você também pode incluir eventos relacionados ao escalonamento:

```bash
perf stat \
    -e task-clock,cycles,instructions,cache-references,cache-misses,branches,branch-misses,context-switches,cpu-migrations \
    -- ./build/mapf <argumentos-do-programa>
```

Muitos context switches, combinados com pouco uso efetivo dos núcleos, podem indicar granularidade pequena demais, excesso de threads, bloqueios ou contenção. Entretanto, um flame graph de CPU não mostra diretamente todo o tempo em que uma thread ficou bloqueada ou dormindo; ele mostra principalmente onde o processo consumiu CPU.

## 13. Comparar antes e depois de uma otimização

Use o mesmo caso de teste e produza dois perfis:

```bash
perf record -F 199 -g --call-graph fp -o perf-before.data -- ./build/mapf <argumentos>
```

Depois da mudança:

```bash
perf record -F 199 -g --call-graph fp -o perf-after.data -- ./build/mapf <argumentos>
```

Compare os relatórios:

```bash
perf diff perf-before.data perf-after.data
```

Além do perfil, meça várias execuções com `perf stat -r 5`. Uma função ocupar uma porcentagem maior depois da otimização não significa necessariamente que ficou mais lenta: outra parte do programa pode ter ficado mais rápida. Sempre compare também o tempo total e, no caso do seu MAPF, métricas algorítmicas como custo da solução, número de iterações e quantidade de expansões.

## 14. Arquivos gerados

| Arquivo | Conteúdo |
| --- | --- |
| `perf-mapf.data` | Perfil binário original coletado pelo `perf` |
| `perf-mapf-report.txt` | Tabela textual com custos acumulados |
| `perf-mapf-self-report.txt` | Tabela textual focada no custo próprio |
| `perf-mapf.stacks` | Pilhas convertidas para texto |
| `perf-mapf.folded` | Pilhas iguais agrupadas para o FlameGraph |
| `mapf-flamegraph.svg` | Flame graph interativo final |

Normalmente, esses artefatos não devem ser versionados no Git. Você pode adicioná-los ao `.gitignore`:

```gitignore
perf*.data
perf*.stacks
perf*.folded
perf*-report.txt
*-flamegraph.svg
```

## Referências

- [Tutorial oficial do Linux `perf`](https://perfwiki.github.io/main/tutorial/)
- [Repositório oficial FlameGraph, de Brendan Gregg](https://github.com/brendangregg/FlameGraph)
- [Página de Flame Graphs de Brendan Gregg](https://www.brendangregg.com/flamegraphs.html)

