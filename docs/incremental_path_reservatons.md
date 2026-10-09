# Reservas incrementais de caminhos

Este documento consolida o diagnóstico de `melhoria_3.md`, as decisões de `melhoria_3_resposta.md`, os ajustes de `melhoria_3_1.md` e o plano de `plano_melhoria_3.md`. As decisões posteriores prevalecem sobre as alternativas da primeira análise. Os documentos originais ficam como histórico; esta é a referência consolidada do problema e das abordagens para melhorar o desempenho.

O [plano aprovado](../specs/008-incremental-path-reservations/plan.md) organiza a implementação, e a [especificação](../specs/008-incremental-path-reservations/spec.md) define seus contratos e critérios de aceitação. O funcionamento geral dos solvers está em [reparo local de caminhos](local_path_repair_parallel_solver.md).

## 1. Problema: reconstruções completas durante o reparo local

`buildReservationState` percorre os caminhos incluídos, registra ocupações e movimentos, une os bloqueios de cada célula e calcula seus intervalos seguros. **Essa função não executa SIPP**: as buscas são realizadas nas chamadas a `sipp.solve`.

Antes desta melhoria, o motor compartilhado de reparo local repetia a construção das reservas mesmo quando apenas um caminho mudava:

| Momento | Trabalho anterior | Abordagem aprovada |
| --- | --- | --- |
| Inicialização | Construir `result.reservations` com todos os caminhos. | Manter uma construção completa inicial. |
| Preparação do reparo | Reconstruir as reservas excluindo o agente ativo. | Registrar as entradas afetadas e remover somente as contribuições desse agente. |
| Fallback do caminho completo | Copiar os demais caminhos e chamar a sobrecarga do SIPP que constrói outra tabela. | Reutilizar a tabela preparada, com política de destino permanente. |
| Aceitação | Reconstruir todas as reservas dos caminhos atualizados. | Proteger as entradas adicionais, inserir o novo caminho no estado e confirmar as alterações. |

Uma tentativa aceita após fallback podia, portanto, provocar três construções completas além da inicial: exclusão, fallback e aceitação. As diferentes âncoras de uma mesma tentativa já compartilhavam a tabela; não havia uma reconstrução por âncora.

O objetivo é eliminar essas reconstruções e recalcular intervalos somente nas células dos caminhos removido e inserido. A primeira implementação manteve uma cópia completa por tentativa. Após o diagnóstico de desempenho, o reparo local passou a usar um registro das entradas afetadas para desfazer alterações, eliminando essa cópia. O solver de reparo completo mantém seu isolamento por cópia.

## 2. Dados necessários para desfazer reservas

Os intervalos seguros são uma informação agregada: não registram quem produziu cada bloqueio. O antigo conjunto de agentes por célula também não informava o instante da passagem. Para excluir um caminho corretamente, é necessário conservar a origem temporal das ocupações em `PathReservationState`:

```cpp
using VertexOccupants = std::unordered_map<Cell*,
    std::unordered_map<int, std::unordered_set<int>>>;

struct GoalReservation {
    int agentId;
    int arrivalTime;
};

struct PathReservationState {
    SafeIntervalTable safeIntervalTable;
    VertexOccupants vertex_agents;
    std::unordered_map<Cell*, GoalReservation> goal_reservations;
};
```

| Componente | Significado |
| --- | --- |
| `vertex_agents[cell][t]` | IDs dos agentes que ocupam explicitamente a célula no instante `t`, incluindo a última posição do caminho. |
| `goal_reservations[cell]` | Proprietário e instante de início da ocupação permanente do destino. |
| `safeIntervalTable.safeIntervalsByCell[cell]` | Complemento normalizado da união das ocupações explícitas e permanentes. |
| `safeIntervalTable.blockedEdgeArrivals[{V,U}]` | Instantes de chegada dos movimentos `U -> V` que impedem uma troca na direção inversa. |

Esses quatro componentes devem permanecer consistentes. Se X e Y ocupam `A` em `5`, excluir X altera o conjunto de proprietários, mas mantém o bloqueio de `A@5`.

`Instance` atribui `Agent::id` conforme a posição no vetor de agentes: `agents[i].id == i`. Na construção manual, os IDs recebidos são substituídos na cópia interna; `scenarioId` e as posições são preservados. Assim, reservas e conflitos usam os mesmos IDs/índices de caminhos. A atualização dos conflitos recebe as reservas por referência constante, sem conversão de IDs ou cópia adicional. Os helpers continuam recebendo explicitamente o ID do agente cujo caminho será removido ou inserido.

A proposta de reutilizar `GoalOccupancy` foi ajustada: esse tipo é privado de `utils.cpp`, e seu campo `i` representa índice de caminho. `GoalReservation` torna a identidade explícita sem alterar o detector. Como os destinos são únicos, basta um proprietário permanente por célula; outros agentes ainda podem visitá-la temporariamente.

As reservas pertencem à execução do solver, sem adicionar estado temporal a `Cell` ou `Grid`. O solver paralelo paraleliza os A* iniciais; o reparo compartilhado continua sequencial. Não é necessário um cache global ou sincronização adicional para esta versão.

## 3. Intervalos fechados e ocupação permanente

O tempo é discreto e os extremos dos intervalos são inclusivos. O exemplo corrigido da discussão é:

```text
Ocupações:                 [5,6], [10,11]
Intervalos seguros:        [0,4], [7,9], [12,+inf]
Após liberar somente t=5:  [0,5], [7,9], [12,+inf]
```

Não basta estender as bordas existentes. Remoções e inserções podem criar, fundir, dividir ou eliminar intervalos:

| Alteração, sem outras ocupações | Resultado correto |
| --- | --- |
| Remover o único bloqueio em `3`, entre `[0,2]` e `[4,+inf]`. | Um intervalo seguro `[0,+inf]`. |
| Remover `6` de um bloqueio `[5,7]`. | Criar o intervalo seguro `[6,6]`. |
| Inserir uma passagem em `5` numa célula livre. | Dividir a liberdade em `[0,4]` e `[6,+inf]`. |
| Remover X, estacionado em `A` desde `4`, mantendo uma visita de Y em `7`. | Intervalos seguros `[0,6]` e `[8,+inf]`. |
| Remover uma visita de X em `5`, mantendo Y estacionado desde `3`. | O instante `5` continua bloqueado. |

A chegada em `t` bloqueia o destino desde `t`, inclusive; o intervalo seguro anterior termina em `t-1`. A última posição explícita e a reserva permanente se sobrepõem intencionalmente. A ocupação efetiva é sua união, sem expandir a permanência em infinitas entradas temporais.

A rotina compartilhada de regeneração de uma célula deve:

1. Reunir `[t,t]` para os conjuntos temporais não vazios e, se houver destino, `[arrivalTime, SAFE_INTERVAL_INFINITY]`.
2. Ordenar os bloqueios e unir sobreposições e adjacências. `unordered_map` não garante ordem temporal.
3. Calcular o complemento a partir de zero, evitando intervalos vazios e aritmética além da sentinela de infinito.
4. Substituir o vetor da célula, com intervalos seguros ordenados e maximais.

Cada operação reúne as células distintas do caminho e regenera cada uma uma única vez, mesmo com esperas e revisitas. Limpar conjuntos de ocupantes vazios não significa apagar entradas livres da tabela: uma célula livre mantém `[0, SAFE_INTERVAL_INFINITY]`; uma totalmente bloqueada possui vetor vazio. Na API atual, uma célula ausente da tabela é indisponível.

## 4. Arestas: direção, instante e multiplicidade

Um movimento `U -> V` que chega em `5` registra `{V,U}` em `blockedEdgeArrivals` no instante `5`. Usa-se o tempo de chegada, e esperas `U -> U` não geram bloqueio de aresta.

Se X e Y fazem esse mesmo movimento, retirar X não pode simplesmente apagar `5`: a restrição de Y permanece. Após remover as visitas de X, o índice temporal permite verificar se existe um mesmo ID em `vertex_agents[U][4]` e `vertex_agents[V][5]`. Uma interseção não vazia indica outro agente realizando o movimento. Assim, não é obrigatório criar um segundo índice de proprietários por aresta.

O estado inicial pode conter colisões; por isso, movimentos compartilhados precisam funcionar mesmo que não apareçam numa solução válida. A chave do SIPP é direcionada, enquanto o detector usa extremidades canônicas e participantes separados por direção. Não se deve substituir uma pela outra sem conversão explícita.

## 5. Exclusão, busca e confirmação

As duas operações alteram `PathReservationState` protegido por um registro de alterações ou isolado em uma cópia descartável. Elas recebem um caminho **completo**, começando em tempo absoluto zero:

```cpp
void repairSafeIntervalTable(
    PathReservationState& state, const std::list<Cell*>& oldPath, int agentId);

void updateReservationState(
    PathReservationState& state, const std::list<Cell*>& newPath, int agentId);
```

Apesar do nome, `repairSafeIntervalTable` atualiza todos os componentes do estado ao remover o caminho. `updateReservationState` adiciona somente o novo caminho; ela exige que a contribuição antiga já tenha sido retirada. Caminhos vazios não contribuem com reservas. Um caminho de uma célula ocupa seu destino permanentemente desde zero.

O fluxo em `repairInitialPaths` é:

```text
construir result.reservations uma vez
para cada tentativa efetiva de reparo:
    registrar entradas afetadas pelo caminho antigo
    remover o caminho completo do agente ativo do estado
    executar buscas usando essa tabela estável
    validar a estrutura do candidato completo
    se rejeitado: restaurar as entradas registradas
    se aceito:
        proteger entradas adicionais e adicionar o novo caminho completo ao estado
        preparar as demais alterações que possam alocar memória
        atualizar os conflitos afetados, incluindo as ocupações permanentes
        publicar caminhos e conflitos e confirmar a transação das reservas
```

Adicionar o novo caminho diretamente ao estado original deixaria reservas obsoletas. Exemplo: a rota antiga passa por `A@5` e a nova por `B@5`; sem exclusão prévia, ambas ficariam reservadas.

Atualizar apenas a janela geométrica também é insuficiente: inserir duas esperas desloca em dois instantes toda a cauda posterior, inclusive sua chegada ao destino. A remoção e inserção dos caminhos completos cobrem as células antigas, as novas e esses deslocamentos.

Durante a tentativa local, `result.reservations` contém alterações provisórias protegidas por `ReservationTransaction`. Cada entrada afetada é registrada uma única vez; os nós originais são preservados e a tentativa trabalha sobre cópias apenas de seus valores. Rejeição, falha no fallback ou exceções removem as entradas provisórias e reinserem os nós originais sem alocação. Os mapas mantêm capacidade para o tamanho anterior. A restauração explícita precede o retorno por falha, e a confirmação ocorre depois das operações que podem alocar memória. Erros de contrato, como proprietário incorreto, segunda remoção ou destino já reservado, continuam sendo detectados.

Todas as buscas de uma tentativa compartilham a tabela preparada, sem alterá-la. SIPP guarda índices e ponteiros para intervalos: se o índice `1` identifica `[4,+inf]`, fundir esse intervalo com `[0,2]` durante a busca eliminaria o índice. Fazer a exclusão antes das buscas e a inserção após seu término resolve esse risco nesta versão, sem cache por revisão.

## 6. Reutilizar a tabela no fallback preservando a política do destino

A sobrecarga de SIPP que recebe caminhos constrói reservas e exige destino permanente. A sobrecarga original que recebe tabela e tempo inicial usa destino transitório. Apenas trocar a chamada do fallback mudaria seu comportamento.

Considere `G` livre em `[0,4]` e `[6,+inf]`, com uma passagem de outro agente em `5`. O agente ativo consegue chegar em `3` ou esperar com segurança fora de `G` e chegar em `6`:

| Política | Resultado esperado |
| --- | --- |
| `Transient` | Pode aceitar chegada em `3`, adequada a um ponto intermediário se o caminho continuar e sair antes de `5`. |
| `Permanent` | Deve buscar chegada em `6` ou depois, pois o agente permanecerá no destino. |

Rejeitar a chegada em `3` apenas na validação posterior pode perder a solução válida que chegaria em `6`. A solução aprovada expõe `GoalOccupation` e acrescenta uma sobrecarga com tabela, tempo e política explícita. As chamadas antigas mantêm seu comportamento; o fallback usa:

```cpp
sipp.solve(grid, start, goal, repairState.safeIntervalTable, 0,
           AStarSippSolver::GoalOccupation::Permanent);
```

Isso remove tanto a reconstrução interna do fallback quanto a cópia dos demais caminhos feita por `pathsExcept`.

## 7. Integração e consumidores afetados

| Local | Responsabilidade |
| --- | --- |
| [local_path_repair_solver.hpp](../include/mapf/solvers/local_path_repair_solver.hpp) | Tipos de ocupação temporal e proprietário do destino. |
| [local_path_repair_solver_common.hpp](../src/mapf/solvers/local_path_repair_solver_common.hpp) e [implementação compartilhada](../src/mapf/solvers/local_path_repair_solver_common.cpp) | Construtor inicial, regeneração por célula, exclusão/inserção e coordenação da publicação. |
| [a_star_sipp.hpp](../include/mapf/pathfinding/a_star_sipp.hpp) e [a_star_sipp.cpp](../src/mapf/pathfinding/a_star_sipp.cpp) | Política pública de destino e busca com tabela já construída. |
| [path_reservation_state_test.cpp](../tests/solvers/path_reservation_state_test.cpp) | Equivalência com reconstrução completa e casos de atualização local. |
| Testes de SIPP e dos solvers iterativo e paralelo | Compatibilidade das sobrecargas, consumidores dos mapas, falhas e estratégias de seleção. |

`hasOtherAgent`, usada na construção de candidatos e no reparo do sufixo, deve percorrer os conjuntos temporais da célula até encontrar um ID diferente do ativo. Essa consulta continua espacial: testa se outro agente visita a célula em algum instante.

Os testes também precisam mudar: `vertex_agents.at(cell).contains(0)` continua compilando, mas agora verifica o **instante zero**, não o agente zero. Reservas de destino devem ser verificadas por `agentId` e `arrivalTime`.

O construtor completo permanece como referência de equivalência. `SolutionConflicts` não substitui o índice de ocupações: ele contém eventos com conflito, omitindo passagens isoladas e movimentos sem troca que ainda restringem SIPP. A validação global dos candidatos permanece necessária, inclusive para conflitos deslocados para fora da janela local.

## 8. Custos e limites do ganho esperado

Se `V` é o número de células, `L` a soma dos comprimentos dos caminhos e `b_c` o número de bloqueios brutos da célula `c`, uma construção completa custa aproximadamente:

```text
O(V + L + soma_c(b_c log b_c))
```

Essa estimativa considera custos esperados dos mapas hash. A abordagem incremental mantém uma construção inicial e troca as reconstruções seguintes por operações locais. O registro para restauração copia somente os valores das entradas afetadas: vetores de intervalos, ocupações temporais das células, conjuntos de tempos das arestas e reservas de destino. Seu custo depende dessas entradas, incluindo reservas de outros agentes que compartilham as mesmas células ou arestas, sem percorrer o estado completo a cada tentativa.

A remoção/inserção percorre o caminho enviado, reúne suas células distintas e consulta as ocupações restantes dessas células e dos extremos dos movimentos afetados. Portanto, seu custo não é somente `O(tamanho do caminho)`: células disputadas podem conter muitas visitas a ordenar, e conjuntos de agentes precisam ser consultados para preservar arestas compartilhadas.

Espera-se reduzir trabalho quando cada reparo afeta poucas células em relação ao mapa. Entretanto, os conjuntos temporais aumentam memória e custo de registro; regiões muito disputadas reduzem a vantagem. Buscas SIPP, cópias de candidatos, cálculo de impressões digitais e atualização de conflitos continuam consumindo tempo. Menos células recalculadas não garante uma redução proporcional no tempo total.

## 9. Validação da correção e do desempenho

O critério central compara os quatro componentes do estado, sem depender da ordem dos mapas hash:

```text
cópia(estado completo) - caminho antigo[i]
    == reconstrução completa excluindo i

(cópia(estado completo) - caminho antigo[i]) + caminho novo[i]
    == reconstrução completa dos caminhos atualizados
```

Além dessa comparação, usar resultados esperados independentes para evitar que um erro na rotina compartilhada apareça dos dois lados. Os casos relevantes são:

- Ocupações compartilhadas e visitas em tempos distintos; esperas e revisitas.
- Fusão, divisão, criação de intervalos unitários e limites inclusivos.
- Remoção do proprietário permanente mantendo visitantes, e remoção de visitantes mantendo o proprietário.
- Movimentos compartilhados, sentidos opostos e tempos de chegada diferentes.
- Desvios para novas células, chegada antecipada/atrasada e cauda deslocada.
- IDs não contíguos, caminhos vazios/unitários e limites da sentinela temporal.
- Remoção com identidade errada, remoção repetida e inserção com destino ocupado.
- Fallback permanente com possibilidade de chegada posterior; tabela estável entre buscas.
- Tentativas rejeitadas, continuação após falha e substituições sucessivas.
- Células e arestas não afetadas preservadas; ambos os adaptadores e ambas as estratégias.

Para medir desempenho, comparar as mesmas instâncias, estratégia, número de trabalhadores, compilador e procedimento de repetição. Separar construção inicial, registro das entradas, exclusão, inserção, SIPP, detecção de conflitos e tempo total. Contar construções completas e células/arestas registradas e regeneradas; registrar também pico de memória. A expectativa estrutural do reparo local é uma construção inicial, um registro das entradas afetadas por tentativa e nenhuma cópia ou reconstrução completa no laço ou fallback. Os resultados históricos abaixo se referem à implementação anterior, que ainda copiava o estado.

Os quatro testes citados na análise original validavam a base anterior à implementação; esse resultado histórico não demonstra correção nem ganho da atualização incremental.

### 9.1. Evidência da implementação em 29/09/2026

A compilação `Release` da implementação e a suíte completa de CTest passaram: **9/9 testes**. A cobertura inclui o novo teste de reservas, comparações com reconstrução completa, resultados esperados independentes, sequências determinísticas de substituição e regressões dos dois solvers e do SIPP.

```bash
cmake -S . -B /tmp/mapf-incremental-build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/mapf-incremental-build -j2
ctest --test-dir /tmp/mapf-incremental-build --output-on-failure
```

A execução adicional com sanitizadores não foi concluída: LeakSanitizer informou incompatibilidade com `ptrace` no ambiente. O resultado acima se refere à suíte normal, sem afirmar validação por sanitizadores.

### 9.2. Medição comparativa

Comparação entre o commit anterior `f61112282f287a3769391ed201064913a4d75bab` e a implementação na árvore de trabalho. Ambiente: Intel Core i5-8265U, GCC 15.2.0, C++20, `Release` com `-O3 -DNDEBUG`, solver iterativo sequencial e `continueIfFailed=true`. Não foram adicionados instrumentos à API pública.

Foram usados quatro cenários sintéticos com componentes desconectados; as células fora dos corredores são obstáculos, mas também possuem entradas na tabela, conforme o construtor do projeto:

| Cenário | Construção |
| --- | --- |
| Mapa compacto | Grid `32×32`, 32 cruzamentos isolados com braços formando corredores de 3 células; 64 agentes, um par perpendicular por cruzamento. |
| Mapa grande | Os mesmos cruzamentos e caminhos num grid `128×128`. |
| Caminhos maiores | Grid `80×80`, 16 cruzamentos isolados com corredores de 17 células; 32 agentes. |
| Fallback sem solução | Grid `128×128`, 32 corredores isolados de duas células, cada um com um par tentando trocar as posições; 64 agentes. |

Os cruzamentos são distribuídos por linhas: oito componentes por linha nos casos curtos e quatro no caso longo, separados por uma célula. Os pares recebem IDs não contíguos `100+6i` e `103+6i`. Cada processo executa um aquecimento e cinco resoluções medidas. Os tempos abaixo são medianas das médias de três processos por versão, alternando a ordem anterior/incremental. O tempo de `solve` foi medido sem instrumentação de fases.

| Cenário | Estratégia | Anterior (ms) | Incremental (ms) |
| --- | --- | ---: | ---: |
| Mapa compacto | Por agente | 23,89 | 10,42 |
| Mapa compacto | Por tempo | 26,14 | 11,49 |
| Mapa grande | Por agente | 379,72 | 56,72 |
| Mapa grande | Por tempo | 397,06 | 58,58 |
| Caminhos maiores | Por agente | 67,71 | 20,12 |
| Caminhos maiores | Por tempo | 67,53 | 20,04 |
| Fallback sem solução | Por agente | 514,01 | 97,87 |
| Fallback sem solução | Por tempo | 523,68 | 107,62 |

Em todos os cenários e estratégias, a impressão digital dos caminhos finais, sucesso/falha, soma de custos e makespan coincidiu entre as versões e entre as repetições.

Em cópias instrumentadas separadas, cinco resoluções após aquecimento confirmaram as mesmas contagens nas duas estratégias. A contagem de células abaixo soma as regenerações de cada operação, sem incluir a construção inicial:

| Cenário | Construções completas anterior → incremental | Cópias do estado na versão incremental | Células regeneradas no laço/fallback anterior → incremental |
| --- | ---: | ---: | ---: |
| Mapa compacto | 65 → 1 | 32 | 65.536 → 192 |
| Mapa grande | 65 → 1 | 32 | 1.048.576 → 192 |
| Caminhos maiores | 33 → 1 | 16 | 204.800 → 544 |
| Fallback sem solução | 129 → 1 | 64 | 2.097.152 → 128 |

No mapa grande, a instrumentação por agente separou estes tempos médios por resolução:

| Fase | Anterior (ms) | Incremental (ms) |
| --- | ---: | ---: |
| Construção inicial | 2,365 | 1,406 |
| Cópia do estado | 0 | 27,862 |
| Exclusão do agente ativo | 127,669 | 0,098 |
| Inclusão após aceitação | 126,310 | 0,091 |
| Busca SIPP | 0,173 | 0,095 |
| Detecção de conflitos | 3,965 | 3,104 |

Nesse cenário não houve fallback. No cenário sem solução, a construção de tabelas dentro do fallback caiu de 64 chamadas e 187,970 ms para zero; a cópia dos demais caminhos caiu de 64 chamadas e 0,357 ms para zero. As 64 cópias do estado incremental consumiram 53,543 ms. Esses tempos instrumentados não compõem integralmente o tempo total: planejamento inicial, destruição de estados, candidatos e coordenação ficam fora das fases listadas.

O pico de RSS foi medido separadamente, com processos iniciados diretamente pelo shell e cinco resoluções após aquecimento, na estratégia por agente:

| Cenário | Anterior (KiB) | Incremental (KiB) |
| --- | ---: | ---: |
| Mapa compacto | 4.692 | 4.684 |
| Mapa grande | 8.752 | 7.524 |
| Caminhos maiores | 6.764 | 6.372 |
| Fallback sem solução | 9.960 | 7.348 |

O RSS inclui todo o processo e depende do alocador; os valores não medem isoladamente o tamanho do índice temporal. A coleta separada evitou o piso de RSS observado ao iniciar as medições pelo controlador Python. Os artefatos desta execução ficaram em `/tmp/mapf-reservations-008-l8qxtuy1`: `measurement.cpp`, `reservation_probe.hpp`, `prepare_measurement.py` e `measurements.json`; são arquivos temporários de medição.

Esses casos mostram a redução de reconstruções, mas favorecem reparos pequenos e independentes; não representam congestionamento intenso nem demonstram ganho universal. As buscas são curtas e o custo de copiar a tabela permanece significativo. O próximo passo de desempenho deve considerar instâncias representativas e a cópia global, antes de acrescentar novos índices.

## 10. Alternativas e melhorias posteriores

| Abordagem | Decisão e cuidados |
| --- | --- |
| Cópia completa com remoção/inserção local | Escolhida para a primeira versão: facilita descarte de tentativas e preserva a API concreta da tabela. |
| Visão com substituições locais (*overlay*) | Adiada. Reutiliza a base e armazena somente entradas alteradas, mas exige adaptar as consultas de SIPP e os auxiliares. Uma substituição vazia deve esconder a base, enquanto uma ausente deve consultá-la. |
| Contadores ou proprietários adicionais por aresta | Não são obrigatórios: a interseção dos conjuntos temporais preserva os movimentos restantes. Um índice próprio pode ser avaliado se essa consulta dominar o custo. |
| Índice espacial adicional | Adiado. Pode acelerar `hasOtherAgent`, mas exige manter a participação enquanto restar alguma visita do agente à célula. |

- TODO: Medir e reduzir a cópia global por tentativa usando uma visão local ou estrutura equivalente, preservando publicação consistente e isolamento de falhas.
- TODO: Evitar regenerar vetores quando a ocupação efetiva não mudar, sem deixar de atualizar seus proprietários.
- TODO: Avaliar diferenças entre caminhos por célula **e tempo absoluto**, preservando trechos realmente inalterados e cobrindo deslocamentos da cauda.
- TODO: Compartilhar a geração de intervalos com o construtor independente do SIPP para reduzir duplicação das regras.
- TODO: Considerar uma tabela esparsa somente após definir explicitamente a semântica de célula ausente; atualmente ela não significa liberdade.
- TODO: Avaliar reutilização entre tentativas do mesmo agente e revisão, com invalidação após qualquer caminho aceito e limite de memória.
- TODO: Remover temporários desnecessários na construção de reservas e medir cópias de candidatos, impressões digitais e detecção global antes de ampliar o escopo incremental.

Essas alternativas preservam as regras do problema: origens e destinos únicos, permanência no destino e proibição de colisões de vértice e trocas de posições. Movimentos de seguimento e ciclos continuam permitidos.
