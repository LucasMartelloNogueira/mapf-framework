# Avaliação da melhoria proposta em `task.md`

A proposta é pertinente: as reservas dos outros agentes permanecem iguais durante uma tentativa de reparo, portanto não é necessário reconstruir seus intervalos em todo o mapa. Entretanto, **recalcular apenas os intervalos das células de `excludedPath` é uma parte da solução**. A implementação também precisa preservar a origem das ocupações, atualizar bloqueios de arestas e destinos permanentes, incorporar o caminho aceito e manter tentativas rejeitadas isoladas do estado confirmado.

Esta análise considera o código presente na árvore de trabalho, incluindo suas alterações locais. Os riscos descritos para a atualização incremental são lacunas da proposta; não significam que a reconstrução completa atual já apresente esses defeitos. A otimização ainda não foi implementada nem teve seu desempenho medido.

## 1. O que efetivamente está sendo repetido

`buildReservationState` **não executa a busca SIPP**. Ela percorre os caminhos, registra ocupações e movimentos, agrupa intervalos bloqueados e calcula seus complementos. A busca ocorre nas chamadas a `sipp.solve`. Essa distinção ajuda a medir se o custo dominante está na preparação das reservas ou na busca.

Os pontos relevantes estão em [local_path_repair_solver_common.cpp](src/mapf/solvers/local_path_repair_solver_common.cpp):

| Ponto | Trabalho atual | Oportunidade |
| --- | --- | --- |
| `buildReservationState`, linha 98 | Percorre todos os caminhos incluídos e todas as células do grid; ordena e une ocupações por célula. | Construir um índice completo uma vez e recalcular entradas afetadas. |
| `repairInitialPaths`, linha 985 | Constrói `result.reservations` inicialmente. | Manter como inicialização. |
| `repairInitialPaths`, linha 1028 | Reconstrói `repairState` excluindo o agente ativo. | Consultar uma visão do estado existente sem esse agente. |
| Fallback, linhas 1049–1050 | Copia os demais caminhos e chama a sobrecarga do SIPP que constrói outra tabela. | Reutilizar a mesma visão de reservas, preservando a política de destino permanente. |
| Aceitação, linha 1070 | Reconstrói todas as reservas de `accepted->paths`. | Atualizar as contribuições do caminho antigo para o novo. |

Assim, uma tentativa que chega ao fallback e é aceita pode provocar **três construções completas de tabelas de reservas**: exclusão, fallback e publicação do resultado. Alterar somente a linha 1028 deixa duas fontes de custo intactas.

As diferentes âncoras de uma mesma tentativa já compartilham `repairState`. Não há uma reconstrução dessa tabela por âncora, embora existam várias buscas SIPP possíveis.

## 2. Problemas e requisitos não contemplados

### 2.1. O exemplo mistura limites inclusivos e exclusivos

No código, os intervalos são fechados: `[início, fim]`, incluindo ambos os extremos. Portanto, ocupações `[5, 6]` e `[10, 12]` produzem:

```text
Antes:                  [0, 4], [7, 9], [13, infinito]
Depois de liberar t=5:   [0, 5], [7, 9], [13, infinito]
```

O intervalo seguro começando em `12`, como aparece em `task.md`, sobrepõe uma ocupação em `12`. Para manter esse início, a segunda ocupação teria de terminar em `11`.

A atualização deve cobrir mais que a extensão de uma borda. Liberar `6` no meio de um bloqueio `[5, 7]` cria o intervalo seguro `[6, 6]`; liberar o único bloqueio entre dois intervalos seguros os une. A inserção do novo caminho faz o inverso: pode encurtar, dividir ou eliminar intervalos seguros.

### 2.2. Os dados atuais perderam a origem dos bloqueios

[PathReservationState](include/mapf/solvers/local_path_repair_solver.hpp) contém `safeIntervalTable`, `vertex_agents` e `goal_reservations`. Nenhum deles, isoladamente, permite desfazer todas as contribuições de um caminho:

- `safeIntervalsByCell` guarda o resultado agregado, sem os agentes responsáveis pelos bloqueios.
- `vertex_agents` informa quais IDs passam por uma célula, mas não quando passam.
- `blockedEdgeArrivals` guarda conjuntos de tempos, sem multiplicidade ou proprietários.
- `goal_reservations` guarda a chegada permanente, mas não identifica explicitamente o proprietário.

Se dois agentes ocupam `A` em `t=5`, remover um deve manter o bloqueio. A presença dos dois em `vertex_agents[A]` não resolve isso: o segundo pode visitar `A` apenas em outro instante.

É necessário manter as **contribuições originais**, e não tentar recuperá-las a partir dos intervalos já unidos. Contadores por célula/tempo seriam suficientes para subtrair um caminho conhecido, com controle rigoroso de inserção e remoção. Como a proposta também deseja consultar os agentes, conjuntos de proprietários por instante são uma opção mais direta, embora consumam mais memória.

### 2.3. A última célula continua ocupada após o fim da lista

O modelo atual mantém o agente no destino. `buildReservationState` registra `[arrivalTime, SAFE_INTERVAL_INFINITY]`, além da posição explícita no último instante.

Por exemplo, excluir um caminho que chega a `G` em `t=4` deve remover sua contribuição permanente a partir de `4`. Liberar somente o elemento de índice `4` deixaria `G` bloqueada indevidamente para os tempos seguintes. Inversamente, se outro agente estacionou em `A` em `t=3`, retirar uma passagem do agente ativo em `A` em `t=5` não libera esse instante, mesmo que a lista do agente estacionado termine em `3`.

Armazenar ocupação permanente separadamente evita expandir infinitos instantes. Ao consultar uma célula, combinar ocupações explícitas com a reserva permanente ainda ativa. Na exclusão, filtrar o proprietário em ambas as fontes.

A chegada aparece tanto na lista quanto no intervalo permanente. Isso é inofensivo na união atual, mas exige uma convenção explícita quando se usam contadores: não deixar uma contribuição residual nem decrementar duas vezes uma única contribuição.

**Restrição já existente:** destinos distintos são exigidos por `Instance` e por `getCollision`. Não há necessidade de suportar vários proprietários permanentes da mesma célula nesta melhoria. Um registro `{pathIndex, arrivalTime}` por destino basta, mantendo a validação atual. Outros agentes ainda podem passar temporariamente por esse destino.

### 2.4. Atualizar somente vértices deixa reservas de arestas incorretas

Para um movimento `U -> V` que chega em `t`, o código reserva o movimento inverso `{V, U}` em `blockedEdgeArrivals`, no instante `t`. Isso impede a troca de posições entre agentes.

Ao excluir um caminho, remover suas contribuições de aresta. Ao inserir o novo, registrar seus novos movimentos. A remoção deve preservar um bloqueio se outro agente também o produz; isso importa porque os caminhos iniciais podem conter colisões, inclusive movimentos iguais no mesmo instante.

Esperas `U -> U` não geram reserva de aresta. O instante é o de **chegada**, e não o de saída. Além disso, `EdgeKey` do SIPP é direcionada, enquanto `EdgeTime` do detector usa extremidades canônicas e separa os participantes por direção. Reaproveitar a chave do detector sem preservar esse sentido pode inverter ou perder restrições.

### 2.5. Excluir um caminho não equivale a confirmar um reparo

Há duas operações diferentes:

1. Preparar uma consulta temporária sem o caminho ativo, para que ele não bloqueie sua própria busca.
2. Substituir definitivamente as reservas do caminho antigo pelas do caminho aceito.

Na segunda operação, as células afetadas pertencem à união dos caminhos antigo e novo. O novo trajeto pode entrar em células que não aparecem em `excludedPath`. É preciso atualizar também arestas, chegada ao destino e participação espacial em `vertex_agents`.

Mesmo que o reparo mude apenas um trecho geométrico, um atraso desloca os tempos do restante do caminho. `buildLocalCandidate` calcula esse deslocamento e copia a cauda além da janela local. Logo, atualizar somente a janela reparada deixa reservas temporais antigas nessa cauda.

Para a primeira versão, é mais simples remover a contribuição do caminho antigo inteiro e adicionar a do novo inteiro. Ainda assim, somente células e arestas desses caminhos precisam ser recalculadas. Uma diferença mais fina pode vir depois, preservando apenas posições e movimentos que continuam iguais **nos mesmos tempos absolutos**.

### 2.6. A assinatura proposta não identifica o proprietário nem a revisão

`grid`, `PathReservationState` e uma `std::list<Cell*>` não explicitam qual agente está sendo excluído, se o caminho ainda pertence à configuração atual ou se já foi removido.

Recomendo informar `excludedPathIndex` e vincular a operação à revisão vigente. O caminho deve vir do próprio estado confirmado, ou ser validado contra ele. Não depender de buscas por igualdade entre listas para descobrir o proprietário.

Há duas identidades no código: os eventos de `SolutionConflicts` usam índices de caminho; `vertex_agents` usa `Agent::id`. Esses valores não são intercambiáveis. Uma instância com IDs `10` e `20` continua tendo índices `0` e `1`. O novo índice pode usar `pathIndex` internamente, convertendo para `Agent::id` ao manter a projeção pública `vertex_agents`.

Para caminhos completos, o primeiro elemento corresponde a `t=0`. Se a API passar a aceitar segmentos, ela precisará de um tempo inicial explícito. A exclusão do agente ativo deve continuar abrangendo seu caminho completo, inclusive destino permanente; excluir apenas a janela local manteria autorrestrições que hoje não existem.

### 2.7. Tentativas rejeitadas precisam preservar o estado confirmado

Hoje `repairState` é independente de `result.reservations`. Se a busca falha, um candidato é rejeitado ou `continueIfFailed` avança para outro conflito, as reservas confirmadas continuam válidas.

Uma função que remova reservas diretamente de `result.reservations` precisa restaurá-las em todos esses caminhos de saída, inclusive exceções. Caso contrário, o próximo agente pode planejar sobre um estado incompleto. Repetir a exclusão em outra âncora também não pode remover duas vezes a mesma contribuição.

A publicação atual prepara as reservas antes de substituir caminhos, conflitos e revisão. A atualização incremental deve preservar essa consistência. Preparar um conjunto de alterações antes de publicá-lo ajuda, mas **não garante sozinho ausência de falhas durante a publicação**: inserções e rehashes em mapas também podem alocar. O desenho deve prever publicação sem alocação, compartilhamento de estruturas imutáveis ou restauração segura.

### 2.8. Copiar todo o estado reduz o alcance da otimização

Uma implementação que comece com `PathReservationState repair = committed` e depois altere poucas células economiza reconstrução e ordenação, mas continua copiando todos os mapas, vetores e conjuntos. Com novos índices de ocupação, essa cópia pode ficar ainda mais cara.

Isso pode servir como etapa intermediária para validar a lógica. Entretanto, o objetivo de trabalhar proporcionalmente à região afetada exige evitar também a cópia completa por tentativa. Passar o estado por valor produz o mesmo problema.

### 2.9. O fallback não pode trocar de sobrecarga sem preservar sua política

Em [a_star_sipp.cpp](src/mapf/pathfinding/a_star_sipp.cpp), `solve(..., otherAgentPaths)` constrói uma tabela e usa `GoalOccupation::Permanent`. Já `solve(..., safeIntervalTable, startTime)` usa `GoalOccupation::Transient`.

Portanto, reutilizar `repairState.safeIntervalTable` no fallback por uma simples troca de chamada altera o comportamento: o agente pode chegar ao destino num intervalo finito e permanecer ali quando outro agente passar depois.

Uma validação posterior pode rejeitar esse candidato, mas isso também pode perder uma solução válida que exigiria chegar mais tarde. A busca precisa usar a política permanente desde o início. Recomendo expor uma operação que receba tabela/visão **e política de destino**, preservando os padrões das chamadas existentes. `GoalOccupation` e `solveWithTable` são privados atualmente.

### 2.10. A tabela precisa permanecer estável durante cada busca

O estado do SIPP identifica um nó por célula e índice do intervalo. Dividir ou unir o vetor de intervalos durante uma busca muda o significado desses índices. `intervalAt` também devolve ponteiros para elementos desses vetores.

Preparar as entradas afetadas antes da busca, ou usar caches que não alterem entradas já consultadas. Não modificar a base enquanto uma visão ou busca ainda depende dela. Vincular caches a `(revision, excludedPathIndex)` e invalidá-los ao aceitar qualquer caminho.

## 3. Onde armazenar e manter as ocupações

O local natural é **`PathReservationState`, por meio de um índice de reservas associado a ele**. Não recomendo adicionar ocupações a `Cell` ou `Grid`: essas estruturas representam o mapa, que pode ser utilizado por execuções distintas. Misturar reservas temporais com o mapa exigiria limpeza entre execuções e dificultaria estados provisórios independentes.

Uma organização possível, sem tornar obrigatórios estes nomes:

| Informação | Representação sugerida | Uso |
| --- | --- | --- |
| Ocupações explícitas por célula | `Cell* -> tempos ordenados -> conjunto de pathIndex` | Descobrir quem ocupa cada instante e reconstruir apenas a célula afetada. |
| Ocupações permanentes | `Cell* -> {pathIndex, arrivalTime}` | Representar permanência sem expandir a cauda infinita. |
| Contribuições de aresta | `EdgeKey direcionada -> tempo de chegada -> conjunto de pathIndex` | Excluir somente o proprietário correto do bloqueio inverso. |
| Participação espacial | Contagem de visitas por `(Cell*, pathIndex)` | Manter `vertex_agents` correto quando há esperas, revisitas ou atualização parcial. |
| Tabela de consulta | `SafeIntervalTable` derivada do índice | Preservar consultas rápidas do SIPP. |
| Validade | Revisão associada ao estado/visão | Impedir reutilização de reservas de uma configuração antiga. |

Tempos ordenados, por exemplo com `std::map`, permitem reconstruir intervalos por varredura. Um mapa hash exige ordenar os tempos ao recalcular a célula, mas pode ter outros custos menores. A escolha deve ser medida; não é necessário decidir por uma estrutura de intervalos complexa já na primeira versão.

Contagens de visitas são especialmente úteis para atualização parcial. Se o agente visita `A` em `2` e `8`, retirar apenas a visita de `2` não deve apagar sua participação em `vertex_agents[A]`. Na remoção integral de um caminho, também é possível atualizar o conjunto uma vez por célula distinta, desde que nenhuma contribuição desse proprietário permaneça.

Sugestão de distribuição no projeto:

| Local | Responsabilidade proposta |
| --- | --- |
| `include/mapf/solvers/local_path_repair_solver.hpp` | Associar o novo índice a `PathReservationState`; manter claros os contratos de `vertex_agents` e `goal_reservations`. |
| Novo módulo de reservas, extraído de `local_path_repair_solver_common.cpp` | Implementar registro/remoção de contribuições e geração dos intervalos seguros. Pode começar como detalhe interno. |
| `buildReservationState` | Inicializar o índice completo e suas projeções usando uma mesma rotina de registro de caminho. |
| Nova `repairReservationState` | Preparar a visão que exclui o proprietário ativo; não publicar mudanças no resultado. |
| Nova `preparePathReplacement` e operação de publicação | Preparar a substituição antigo/novo e atualizar apenas entradas afetadas quando o candidato for aceito. |
| `repairInitialPaths` | Coordenar a visão temporária, buscas, validação, publicação e revisão. |
| `a_star_sipp.hpp/.cpp` e leitores `intervalAt`/`edgeBlocked` | Aceitar a abstração de consulta escolhida e permitir destino permanente com tabela preexistente. |

O padrão atual de nomes de funções é `lowerCamelCase`; `repairReservationState` se alinha melhor ao código do que `RepairReservationState`. Mais importante que o nome é definir se a operação cria uma visão, altera um estado ou prepara uma substituição.

Se for desejável não aumentar a API pública nem o peso de `LocalPathRepairResult`, o índice mutável pode ficar em um gerenciador interno de reservas durante `repairInitialPaths`, publicando no resultado somente as projeções atuais. Nesse caso, o gerenciador e o estado retornado precisam ter uma relação explícita, evitando duas fontes independentes de verdade.

## 4. Estratégia recomendada para a consulta e a atualização

Para evitar a cópia global e manter tentativas isoladas, recomendo uma **visão de reservas com substituições locais**:

1. A base contém a configuração confirmada e seu índice completo.
2. Ao excluir o agente ativo, reunir as células distintas e os movimentos de seu caminho.
3. Para cada célula afetada, calcular os intervalos considerando somente contribuições dos demais agentes, incluindo ocupações permanentes. Para arestas afetadas, filtrar o proprietário e preservar os outros.
4. Guardar apenas essas entradas substitutas. Consultas a outras células e arestas reutilizam a base.
5. Reutilizar essa visão nas âncoras, nos reparos do sufixo e no fallback, com a política de destino adequada.
6. Se o candidato falhar, descartar a visão. Se for aceito, preparar as contribuições antigo/novo e publicar caminhos, conflitos, reservas e revisão de forma consistente.

**Uma substituição vazia é diferente de uma substituição ausente.** Se uma aresta perde seu último bloqueio, a visão deve retornar “sem bloqueio”, em vez de voltar à entrada antiga da base. Uma célula totalmente ocupada tem vetor de intervalos seguros vazio; uma célula livre tem `[0, infinito]`.

A assinatura conceitual poderia receber `const PathReservationState&`, `excludedPathIndex`, o caminho completo confirmado e a revisão, retornando `RepairReservationView`. `Grid` pode continuar necessário para validação ou busca, mas não deveria provocar nova varredura global durante a exclusão.

Essa visão **não é diretamente compatível com a API atual**: o SIPP e as funções auxiliares acessam os mapas concretos de `SafeIntervalTable`. Será necessário centralizar consultas a intervalos e bloqueios de aresta em uma interface ou adaptador. Caso essa alteração seja grande para a primeira entrega, uma cópia integral seguida de atualização local é uma etapa válida, documentando que ela ainda tem custo global.

Não é necessário paralelizar essa atualização. O solver paralelo executa os A* iniciais em trabalhadores e depois chama o mesmo `repairInitialPaths` sequencial. Um estado por execução atende aos dois adaptadores; um cache global ligado ao grid acrescentaria problemas desnecessários de compartilhamento.

## 5. Ganho esperado e outras melhorias possíveis

Sejam `V` o número de células, `S` a soma dos comprimentos dos caminhos e `b_c` o número de intervalos bloqueados brutos acumulados na célula `c`. A reconstrução atual custa aproximadamente:

```text
O(V + S + soma_c(b_c log b_c))
```

Essa estimativa considera custos esperados das tabelas hash e inclui a ordenação por célula. Alocações, cópias e construção dos conjuntos também pesam no tempo real.

Com um índice por célula e atualização local, a operação passa a depender dos comprimentos dos caminhos alterados e das ocupações nas células/arestas afetadas. **Não é correto prometer apenas `O(tamanho de excludedPath)`**: reconstruir uma célula muito disputada pode exigir percorrer muitas ocupações de outros agentes. Se os tempos estiverem em mapas hash, existe ainda o custo de ordená-los. No pior caso, a região afetada pode abranger quase todo o trabalho original.

O benefício esperado é maior em mapas grandes com caminhos relativamente curtos, reparos concentrados e muitas tentativas. O índice com proprietários aumenta o consumo de memória; a visão local pode compensar parte desse custo ao evitar cópias completas, mas isso precisa ser medido.

| Melhoria adicional | Avaliação |
| --- | --- |
| Reutilizar a tabela no fallback | Prioridade alta: remove cópia dos demais caminhos e reconstrução redundante. Exige preservar `Permanent`. |
| Atualizar reservas após aceitação | Prioridade alta: necessária para eliminar a segunda chamada a `buildReservationState` no laço. |
| Unificar a geração de intervalos | `buildReservationState` e `AStarSippSolver::getSafeIntervalsByCell` duplicam regras de união/complemento, destinos e arestas. Compartilhar a lógica reduz divergências futuras. |
| Evitar temporários desnecessários | Os construtores convertem cada lista em vetor e copiam os intervalos brutos por célula. É possível percorrer a lista com célula anterior e usar referências onde seguro. Ganho independente e menor que a mudança principal. |
| Recalcular uma vez por célula distinta | Esperas e revisitas devem alterar contribuições por instante, mas disparar uma única reconstrução da célula ao final do lote. |
| Não invalidar intervalos quando a ocupação efetiva não mudou | Retirar um entre vários ocupantes pode manter os mesmos intervalos. Atualizar os proprietários continua necessário, mas o vetor derivado pode ser reutilizado. |
| Representação esparsa de células sempre livres | Pode evitar tabelas para todo o mapa. Hoje, porém, ausência no mapa significa indisponibilidade para SIPP e auxiliares; mudar isso exige um contrato explícito para “livre por padrão”. |
| Reutilização entre tentativas da mesma revisão | Uma visão pode ser reutilizada para o mesmo agente enquanto a configuração não mudar. Limitar o cache evita materializar uma cópia da tabela por agente. |
| Reduzir cópias e detecção global de conflitos | `evaluateCandidate` copia todos os caminhos, executa `getCollision` e calcula uma impressão digital global. Esses custos permanecem após otimizar reservas e podem se tornar dominantes. Medir antes de ampliar o escopo. |

`SolutionConflicts` não pode substituir o índice de reservas: `getCollision` monta ocupações completas em estruturas locais, mas retorna apenas eventos com conflito. Passagens isoladas e arestas sem troca desaparecem da saída, embora sejam necessárias ao SIPP. Pode haver compartilhamento futuro da indexação, mas isso exige mudar o desenho; não basta reutilizar `remainingConflicts`.

Também não se deve descartar a validação global dos candidatos como parte desta otimização. O reparo pode deslocar a cauda e mudar conflitos fora da janela local. Atualização incremental do detector é uma melhoria separada, com requisitos próprios de descoberta dos eventos afetados.

## 6. Ordem sugerida de implementação

1. Definir os invariantes: intervalos fechados, tempos absolutos, identidade por índice, reservas permanentes, sentido das arestas e estado confirmado por revisão.
2. Extrair e manter a reconstrução completa como referência; introduzir o índice de contribuições e conferir se ele produz os mesmos intervalos e projeções.
3. Implementar a exclusão temporária com atualização local, cobrindo também arestas e destino. Uma primeira versão com cópia pode facilitar a comparação, sem ser tratada como resultado final de desempenho.
4. Permitir SIPP com tabela/visão e política de destino explícita; reutilizar as reservas no fallback.
5. Implementar a substituição antigo/novo após aceitação, preservando isolamento de falhas e consistência da revisão.
6. Eliminar cópias globais de reservas com a visão local ou estrutura equivalente; medir tempo e memória nas mesmas instâncias antes e depois.

## 7. Verificação necessária e evidência desta análise

O principal critério de correção é a equivalência com uma reconstrução completa:

```text
visão que exclui i
    == buildReservationState(grid, agents, caminhos, i)

estado após substituir caminho i
    == buildReservationState(grid, agents, caminhos atualizados)
```

Comparar intervalos normalizados, tempos/direções de aresta, participação espacial e chegadas permanentes, além do índice de origem novo. A igualdade é semântica; a ordem de iteração de mapas hash não importa. Na visão, aplicar a comparação aos campos que ela expuser; o estado completo confirmado deve preservar todas as projeções públicas.

| Caso | Resultado a verificar |
| --- | --- |
| Dois ou mais agentes na mesma célula/tempo | Excluir um mantém o bloqueio dos restantes. |
| Agentes na mesma célula em tempos distintos | Liberar somente os instantes que perderam o último ocupante. |
| Esperas e revisitas | Não perder participação espacial enquanto restar uma visita; não reconstruir a mesma célula repetidamente. |
| Liberação de borda, meio e separador de intervalos | Extensão, criação e união dos intervalos seguros corretas. |
| Exclusão de um agente estacionado | Remover a contribuição permanente inteira, preservando passagens de outros agentes. |
| Exclusão de um visitante de destino alheio | Preservar a ocupação permanente do proprietário. |
| Movimento repetido por vários agentes e troca de posições | Preservar multiplicidade, direção inversa e tempo de chegada corretos. |
| Caminho novo fora da rota antiga | Criar reservas nas novas células e remover as obsoletas. |
| Chegada antecipada ou atrasada, com cauda deslocada | Atualizar todos os tempos alterados e a reserva permanente do destino. |
| Candidato rejeitado, exceção e `continueIfFailed` | Estado confirmado permanece intacto; próxima tentativa usa todos os outros agentes. |
| Fallback com destino livre cedo e ocupado depois | Buscar chegada em intervalo permanente, em vez de aceitar o primeiro intervalo finito. |
| IDs não contíguos | Excluir o índice correto sem confundi-lo com o ID público. |
| Caminho vazio, um único elemento e `t=0` | Preservar comportamento diagnóstico e permanência desde zero quando início e destino coincidem. |
| Limites temporais | Não expandir `SAFE_INTERVAL_INFINITY` nem produzir conversões ou aritmética inválidas. |
| Buscas sucessivas e ambas as estratégias | Nenhuma reserva ou visão antiga vaza entre revisões ou execuções. |

Para desempenho, separar construção inicial, criação da visão de exclusão, atualização após aceitação, fallback, busca SIPP, detecção de conflitos e tempo total. Registrar também células recalculadas, entradas copiadas e pico de memória. Usar as mesmas instâncias, estratégia, número de threads e configuração de compilação; ganho de preparação não implica ganho proporcional do solver inteiro.

Nesta avaliação, compilei os alvos relevantes em `/tmp/mapf-melhoria3-review`, com `Debug` e `BUILD_TESTING=ON`, e executei:

```bash
ctest --test-dir /tmp/mapf-melhoria3-review --output-on-failure \
  -R '^(a_star_sipp_test|utils_collision_test|local_path_repair_parallel_solver_test|local_path_repair_iterative_solver_test)$'
```

Os **quatro testes passaram**. Eles incluem verificações existentes de políticas de destino, bloqueios de aresta, reservas obsoletas, IDs não contíguos e falha de reparo. Isso confirma a base examinada nesses alvos; não valida uma implementação incremental ainda inexistente nem demonstra ganho de desempenho. Nenhum arquivo de implementação ou teste foi alterado para produzir esta análise.
