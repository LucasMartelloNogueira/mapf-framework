# Nova abordagem de reparo local e atualização de conflitos

A abordagem atual substitui um trecho do caminho por uma ponte calculada pelo SIPP, reaproveita o restante do caminho antigo e atualiza os conflitos afetados pela alteração. Conflitos que continuam existindo ou que surgem depois da reconexão ficam disponíveis para as próximas iterações.

Um reparo local aceito pode deixar o caminho completo com conflitos. A ponte é planejada respeitando as reservas dos outros agentes, mas o sufixo reaproveitado pode passar a ocupar células e atravessar arestas em outros instantes. A atualização incremental identifica essas consequências antes da próxima seleção de conflito.

O fluxo é compartilhado por `LocalPathRepairIterativeSolver` e `LocalPathRepairParallelSolver`. A diferença entre eles está na geração dos caminhos iniciais com A*: sequencial no primeiro e paralela no segundo. Em ambos, os reparos são executados sequencialmente por `repairInitialPaths`.

## 1. Estado utilizado pelo reparo

O solver mantém os caminhos iniciais, os caminhos correntes, os conflitos restantes e as reservas correspondentes aos caminhos correntes.

| Estrutura | Responsabilidade |
| --- | --- |
| `result.paths` | Caminhos correntes, na ordem dos agentes da instância |
| `result.remainingConflicts` | Eventos de conflito e registros ordenados por agente |
| `vertex_agents[cell][time]` | Agentes presentes explicitamente em uma célula naquele instante |
| `goal_reservations[cell]` | Dono do destino e instante a partir do qual permanece nele |
| `safeIntervalTable` | Intervalos seguros das células e bloqueios de movimentos em sentidos opostos |
| `visitedConfigurations` | Configurações completas de caminhos já aceitas, para impedir sua repetição |

Cada posição de um caminho completo corresponde a um instante absoluto: a primeira célula está em `t=0`. Duas células consecutivas iguais representam uma espera. O custo do caminho é a quantidade de elementos menos um.

As reservas usam os IDs reais dos agentes. Os conflitos usam os índices dos caminhos no vetor. Quando esses valores diferem, `repairInitialPaths` converte os IDs de uma cópia das reservas antes de chamar `UpdateSolutionConflictsV2`.

## 2. Seleção do conflito e preparação das reservas

As estratégias disponíveis são:

- `RESOLVE_BY_AGENT`: seleciona o primeiro registro do primeiro agente com conflitos, seguindo a ordem da instância.
- `RESOLVE_BY_TIME`: compara os primeiros registros dos agentes e seleciona o que vem primeiro na ordenação temporal dos conflitos. Empates completos preservam a ordem da instância.

Depois da seleção, o solver copia `result.reservations` e remove dessa cópia o caminho completo do agente ativo por meio de `repairSafeIntervalTable`. A remoção inclui suas visitas explícitas, sua contribuição aos bloqueios de arestas e sua reserva permanente de destino.

Todas as tentativas de ponte e o eventual fallback usam essa mesma tabela preparada. Os caminhos dos outros agentes permanecem fixos durante a tentativa. O estado confirmado só é substituído depois que o novo caminho, suas reservas e seus conflitos estiverem preparados.

## 3. Âncora, reconexão e busca da ponte

A âncora é a célula preservada de onde a busca local começa. A reconexão é a célula do caminho antigo à qual a ponte deve chegar para permitir reaproveitar o sufixo.

| Conflito selecionado no instante `t` | Âncora inicial | Índice de reconexão no caminho antigo |
| --- | --- | --- |
| Vértice | `t - 1` | `t + 1` |
| Aresta, com chegada em `t` | `t - 1` | `t` |

No conflito de vértice, a substituição contorna a ocorrência conflitante da célula. No conflito de aresta, substitui o movimento que chegava em `t`.

O próximo conflito do agente não limita a reconexão. Por isso, conflitos consecutivos não invalidam automaticamente a tentativa local. Se a célula de reconexão estiver ocupada no instante antigo, o SIPP pode encontrar outra chegada segura.

Se uma tentativa não produzir um candidato utilizável, o solver recua a âncora por divisões inteiras por dois:

```text
anchor, floor(anchor / 2), floor(anchor / 4), ..., 0
```

Por exemplo, uma âncora inicial em 9 gera as tentativas `9, 4, 2, 1, 0`. A célula de reconexão continua sendo a mesma em todas elas. A busca recebe `startTime=anchor`, pois esse é o instante absoluto da célula inicial da ponte.

A política de ocupação do alvo é explícita:

- `GoalOccupation::Transient` quando ainda existe sufixo depois da reconexão. A ponte pode terminar em um intervalo seguro finito.
- `GoalOccupation::Permanent` quando a reconexão é o último elemento do caminho. Nesse caso, a busca deve chegar ao destino em um intervalo que permita permanecer nele.

Uma reconexão inexistente, como a solicitada depois de um conflito de vértice no último índice, leva ao fallback completo. A tentativa local também exige um instante de conflito maior que zero e contido no caminho explícito.

## 4. Construção do caminho e deslocamento do sufixo

Com uma ponte utilizável, o novo caminho é montado assim, usando intervalos inclusivos:

```text
newPath = oldPath[0 .. anchor]
        + bridge[1 .. último]
        + oldPath[reconnectIndex + 1 .. último]
```

A primeira célula da ponte já está no prefixo. A última já representa a reconexão. Cada extremo aparece uma única vez na concatenação, evitando esperas acidentais.

O índice da reconexão no caminho antigo e seu novo instante podem ser diferentes:

```text
newReconnectIndex = anchor + bridge.size() - 1
delta = newReconnectIndex - reconnectIndex
```

Cada célula do sufixo copiado passa a ser visitada `delta` instantes antes ou depois de sua visita antiga. Não se insere uma espera automática para recuperar o horário original, nem se executam mini-buscas ao longo desse sufixo.

Exemplo ilustrativo: um conflito em `X`, no instante 1, é contornado por uma ponte `A → U → V → W → B`.

| Instante | Caminho antigo | Caminho após o reparo |
| --- | --- | --- |
| 0 | A | A |
| 1 | X | U |
| 2 | B | V |
| 3 | C | W |
| 4 | G | B |
| 5 | — | C |
| 6 | — | G |

`G` é o destino: depois de chegar, o agente permanece nele, mesmo sem novos elementos na lista. Nesse exemplo, a reconexão em `B` muda de 2 para 4; o sufixo e a chegada ao destino atrasam dois instantes. Um conflito antigo em `C@3` pode desaparecer e um novo conflito em `C@5` pode surgir. Esses eventos são reavaliados pela atualização incremental.

Antes de aceitar o candidato, o solver verifica a estrutura: caminho não vazio, extremos corretos, células livres, movimentos entre vizinhos ou esperas e tamanho dentro do limite temporal suportado. Também rejeita uma configuração completa de caminhos já visitada. A ausência de colisões no sufixo não é uma condição de aceitação dessa etapa.

## 5. Fallback completo e falha

Se nenhuma âncora produzir um candidato local utilizável, o solver tenta SIPP completo para o agente ativo:

```text
origem: primeira célula do caminho antigo
destino: última célula do caminho antigo
instante inicial: 0
ocupação do destino: GoalOccupation::Permanent
reservas: tabela preparada sem o caminho antigo do agente ativo
```

O fallback também é usado quando não existe uma reconexão local válida. Seu caminho precisa passar pelas verificações estruturais e pelo controle de repetição de configurações.

Se for aceito, a atualização dos conflitos começa em zero, porque todo o caminho pode ter mudado. Se o fallback também falhar, o solver retorna falha com o último estado confirmado.

Não existe mais `continue_if_failed`, nem o conjunto `ignoredForRevision`. O solver não ignora uma falha definitiva para tentar outros conflitos. O fallback mantém os demais caminhos fixos; sua falha não demonstra que seria impossível resolver a instância alterando outros agentes.

## 6. Atualização incremental dos conflitos

Depois de montar o candidato, `updateReservationState` insere seu caminho completo na cópia preparada das reservas. Somente então `UpdateSolutionConflictsV2` recebe o novo caminho, as reservas atualizadas e o conjunto anterior de conflitos.

### 6.1. Onde começa a atualização

No reparo local, `repairStartIndex` recebe a âncora utilizada. No fallback completo, recebe zero.

Esse índice não é a primeira célula após a ponte. A função também precisa remover conflitos antigos do trecho substituído, inclusive em células e arestas que o novo caminho deixou de visitar.

O prefixo até a âncora permanece inalterado. Por isso, a atualização considera vértices a partir de `pathIndex` e movimentos com chegada depois de `pathIndex`. A primeira aresta que pode ter mudado liga os índices `pathIndex` e `pathIndex + 1`.

### 6.2. Quais eventos são reavaliados

A função reúne os eventos afetados antes de atualizar seus registros:

1. Conflitos antigos de `byAgent[agentId]` no intervalo temporal afetado, mesmo quando suas células ou arestas não aparecem mais no caminho novo.
2. Eventos globais de vértice que envolviam o agente ativo nesse intervalo, incluindo sua participação como dono estacionado de um destino.
3. Vértices e movimentos do novo caminho a partir do início da atualização.
4. Visitas explícitas ao destino do agente ativo a partir desse início, inclusive em instantes posteriores ao fim de seu caminho novo.

O quarto grupo permite detectar consequências da antecipação ou do atraso da chegada ao destino nos caminhos dos outros agentes. Os eventos fora do conjunto afetado permanecem preservados.

### 6.3. Conflitos de vértice e permanência no destino

Para cada par célula/instante afetado, a função reúne os ocupantes explícitos de `vertex_agents` e acrescenta o dono de `goal_reservations` quando ele já chegou. Dois ou mais participantes formam um conflito de vértice.

Há dois casos importantes de permanência:

**O agente reparado visita o destino de outro agente.** A presença do dono estacionado precisa ser considerada mesmo que seu caminho explícito já tenha terminado. A ausência de uma visita explícita dele naquele instante não significa que a célula esteja livre.

**O agente reparado muda seu próprio instante de chegada ao destino.** Isso pode alterar conflitos com visitas de outros agentes. Por exemplo, se antes chegava em 176 e agora chega em 186, uma visita ao seu destino em 185 pode deixar de conflitar. Se passa a chegar antes dessa visita, um conflito pode surgir. A ocupação efetiva dos caminhos atualizados determina o resultado.

Os eventos globais incluem todos os participantes, inclusive donos estacionados. Em `byAgent`, somente os ocupantes explícitos recebem o registro. Assim, após o fim do caminho do dono estacionado, o registro que orienta o reparo fica com o visitante. No instante da chegada, o dono ainda ocupa explicitamente a última célula e pode receber o registro.

### 6.4. Conflitos de aresta

Uma troca de aresta em `t` exige movimentos explícitos nos dois sentidos: um agente deve estar em `U` no instante `t-1` e em `V` no instante `t`, enquanto outro faz `V → U` no mesmo intervalo.

A função reconstrói os participantes de cada sentido por interseções dos conjuntos de ocupantes explícitos nos extremos. Arestas canônicas organizam os eventos, mas os dois sentidos permanecem separados. Esperas não são deslocamentos entre extremos diferentes, e a ocupação permanente não cria movimentos virtuais.

### 6.5. Resultado da reavaliação

| Situação do evento | Atualização |
| --- | --- |
| Não havia conflito e continua sem conflito | Nenhum registro ativo é mantido |
| Havia conflito e ele desapareceu | Remove o evento e os registros afetados |
| Não havia conflito e ele surgiu | Adiciona o evento e os registros dos participantes pertinentes |
| O conflito continua | Atualiza seus participantes e, para arestas, os sentidos |

Os registros por agente permanecem ordenados e sem duplicações. Um conflito que perde o agente reparado pode continuar existindo entre outros participantes; a atualização preserva esse evento com a composição correta.

Essa etapa atualiza os registros de conflitos. Ela não modifica novamente os caminhos nem tenta resolver todos os eventos recém-descobertos na mesma chamada.

## 7. Confirmação e próxima iteração

Depois da atualização incremental, o solver registra a configuração aceita e publica juntos o novo caminho, os conflitos atualizados e as reservas preparadas. A próxima seleção utiliza esse novo estado.

O fluxo principal pode ser resumido pelo pseudocódigo:

```text
inicializar caminhos, conflitos e reservas
registrar a configuração inicial

enquanto houver um registro de conflito selecionável:
    escolher agente e conflito
    copiar reservas e remover o caminho completo desse agente

    tentar pontes locais e concatenar prefixo + ponte + sufixo
    se não houver candidato utilizável:
        tentar SIPP completo com destino permanente
    se ainda não houver candidato utilizável:
        retornar falha com o estado confirmado

    inserir o novo caminho nas reservas preparadas
    atualizar conflitos antigos e novos afetados pela alteração
    registrar a configuração e publicar caminho, conflitos e reservas

finalizar o resultado com o estado corrente
```

A aceitação de um reparo não exige que a quantidade total de conflitos diminua imediatamente. Um desvio pode eliminar um conflito e criar outros no sufixo. O controle de configurações impede retornar exatamente a um conjunto de caminhos já aceito; ele não substitui a atualização dos conflitos nem demonstra que toda instância solucionável será resolvida.

## 8. Relação com a abordagem anterior e com o código

`buildLocalCandidate` e `RepairWindow` foram removidos. A construção do caminho não usa mais uma janela encerrada antes do próximo conflito, esperas para restaurar o horário do sufixo ou validações temporais de cada transição reaproveitada. Os elementos que continuam necessários são a âncora e o índice de reconexão.

As responsabilidades estão distribuídas assim:

| Componente | Papel na abordagem atual |
| --- | --- |
| [`repairInitialPaths`](../src/mapf/solvers/local_path_repair_solver_common.cpp) | Seleciona conflitos, prepara reservas, busca pontes, concatena caminhos, tenta fallback e confirma o estado |
| [`AStarSippSolver`](../src/mapf/pathfinding/a_star_sipp.cpp) | Planeja segmentos com instante inicial e política de ocupação do alvo explícitos |
| [`UpdateSolutionConflictsV2`](../src/mapf/utils.cpp) | Reavalia os eventos afetados pelo novo caminho e pela mudança de ocupação permanente |
| [`PathReservationState`](../include/mapf/solvers/local_path_repair_solver.hpp) | Representa visitas explícitas, destinos permanentes e restrições utilizadas pelo SIPP |

A referência detalhada das operações de remoção e inserção de reservas está em [Reservas incrementais de caminhos](incremental_path_reservatons.md). Esta documentação descreve o fluxo implementado; a revisão de `getCollision` e da coerência das métricas permanece fora desta alteração.
