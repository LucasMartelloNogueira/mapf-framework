# Avaliação das propostas para `getCollision`

Este documento avalia as sugestões de `task.md` e `ref.md` para substituir a implementação atual de `getCollision` por uma versão modificada de `getCollisionV1`.

A conclusão geral é que uma versão modificada da V1 é viável, mas as sugestões ainda têm problemas de correção e exigem uma mudança deliberada no contrato atual de `getCollision`.

## Avaliação de `task.md`

### 1. Autoconflito

A sugestão é correta, com uma ressalva. Armazenar o índice do caminho junto com `arrivalTime` permite ignorar a própria reserva do agente e resolve o autoconflito criado pela pré-população do mapa de destinos.

A chave deveria preferencialmente ser `Cell*`, em vez de uma string contendo as coordenadas. As demais partes do projeto já identificam as células da mesma `Grid` por seus ponteiros, e isso evita a criação e o hashing de strings no trecho crítico.

### 2. Flag para impedir conflitos duplicados

Uma flag local não é suficiente para garantir que cada ocorrência seja produzida uma única vez. Ela pode resolver uma duplicação simples entre `goalVertexLookup` e `vertexLookup`, mas falha em situações como:

- dois agentes em movimento encontram um terceiro agente já parado;
- o dono do destino é processado depois do outro agente;
- três ou mais agentes ocupam a mesma célula;
- a mesma ocorrência é descoberta durante o processamento de caminhos diferentes.

O problema deve ser resolvido no nível da ocorrência: primeiro devem ser agregados os ocupantes de `(Cell*, time)` e, depois, devem ser produzidos os registros correspondentes. Uma flag durante o processamento de um único caminho não garante unicidade global.

Também é importante distinguir:

- `arrivalTime == time`: o dono do destino ainda aparece explicitamente no caminho naquele instante, portanto `vertexLookup` já cobre sua ocupação;
- `arrivalTime < time`: trata-se da ocupação virtual que acontece depois do fim do caminho.

Consultar destinos apenas quando `arrivalTime < time` reduz a sobreposição entre os dois mecanismos, mas a agregação dos participantes continua necessária.

### 3. Não admitir origens ou destinos repetidos

Atualmente essa premissa não é garantida globalmente. `Instance` valida os identificadores dos agentes e se as células são livres, mas seu construtor não rejeita origens ou destinos repetidos. O solver detecta esses casos posteriormente, inclusive depois de já chamar `getCollision`. Existem testes explícitos para origens e destinos repetidos.

Além disso, `getCollision` recebe somente caminhos, não uma `Instance`. Isoladamente, a função não consegue verificar se os destinos dos agentes são únicos.

Essa simplificação só seria segura se a unicidade fosse transformada em uma pré-condição efetivamente validada antes de todas as chamadas de `getCollision`. Caso contrário, a estrutura de destinos precisa aceitar vários registros por célula.

Referências relevantes:

- [`src/mapf/core/instance.cpp`](src/mapf/core/instance.cpp), validação feita pelos construtores de `Instance`;
- [`src/mapf/solvers/local_path_repair_solver_common.cpp`](src/mapf/solvers/local_path_repair_solver_common.cpp), ordem em que `getCollision`, `hasSharedStarts` e `hasDuplicateGoals` são executados;
- [`tests/solvers/local_path_repair_parallel_solver_test.cpp`](tests/solvers/local_path_repair_parallel_solver_test.cpp), cenários de origens e destinos repetidos.

### 4. Um conflito por agente

A representação é uma alternativa válida e pode ajudar o reparo local, porque permite consultar diretamente os conflitos de um caminho. Entretanto, ela muda o contrato atual, que exige um registro por par de agentes.

O teste em que três agentes ocupam o mesmo vértice exige três registros, um para cada par. Adotar um registro por agente exige alterar conscientemente:

- `SolutionConflicts`;
- `LocalPathRepairResult`;
- os consumidores do reparo local;
- os testes de colisão;
- a especificação da funcionalidade.

Referências relevantes:

- [`specs/004-local-path-repair-parallel-solver/spec.md`](specs/004-local-path-repair-parallel-solver/spec.md), contrato de emissão por par;
- [`tests/utils_collision_test.cpp`](tests/utils_collision_test.cpp), expectativa de três conflitos para três agentes.

### 5. Multiplicidade nos conflitos de aresta

Um contador simples associado a uma aresta canônica não é suficiente. Para conflitos de aresta, é necessário manter a direção dos movimentos.

Considere o seguinte caso:

- três agentes fazem `A -> B`;
- um agente faz `B -> A`.

Inicialmente, todos estão envolvidos em trocas de aresta. Se o único agente que faz `B -> A` for reparado, restarão três agentes atravessando a aresta no mesmo sentido. Um contador total teria valor `3`, embora não houvesse mais conflito de aresta.

Devem ser mantidos pelo menos dois grupos:

- participantes de `A -> B`;
- participantes de `B -> A`.

O conflito existe somente quando ambos os grupos são não vazios.

### 6. Esperas classificadas como conflitos de aresta

A sugestão é correta. A V1 deve ignorar uma travessia quando `previousCell == cell`. Isso evita que duas esperas na mesma célula sejam interpretadas como movimentos em sentidos opostos.

A implementação atual já exige que os dois agentes tenham se movido antes de registrar uma troca de aresta, e essa semântica deve ser preservada.

### 7. Conflitos posteriores ao fim de ambos os caminhos

A simplificação é correta apenas sob uma pré-condição forte. Se destinos repetidos forem realmente impossíveis, dois agentes terminados não poderão permanecer na mesma célula. Nesse domínio restrito, não será necessário continuar gerando conflitos entre dois agentes já finalizados.

Hoje essa pré-condição não pertence ao contrato genérico de `getCollision`, e caminhos com o mesmo destino podem ser passados diretamente para a função. A mudança depende de formalizar e validar a nova restrição.

## Avaliação de `ref.md`

### Representação de um conflito por agente

A opção 2, com um conflito por agente, faz sentido para acelerar a consulta do reparo local. Entretanto, não é seguro representar o estado apenas como:

```text
agente -> lista de conflitos
evento -> quantidade de agentes
```

O contador isolado perde informações necessárias:

- não informa quais agentes ainda participam do evento;
- não permite uma remoção idempotente;
- não distingue os dois sentidos de um conflito de aresta;
- pode ficar inconsistente quando um reparo remove diversos conflitos e cria outros;
- não permite saber se o novo caminho do agente ainda participa da mesma ocorrência.

Depois de um reparo, todo o sufixo do caminho pode mudar. Portanto, seria necessário remover a participação do agente de todos os eventos antigos afetados, inserir os eventos do novo caminho e reavaliar as colisões resultantes. Decrementar apenas o evento que motivou o reparo não mantém o estado correto.

Uma representação mais segura seria:

```text
evento de vértice -> conjunto de pathIndexes participantes
evento de aresta  -> participantes A->B e participantes B->A
pathIndex         -> referências para seus eventos
```

Com essa representação:

- a quantidade é derivada do tamanho dos conjuntos;
- remover um agente é uma operação segura;
- uma aresta só está em conflito quando existem participantes nos dois sentidos;
- a lista por agente permite encontrar rapidamente seu próximo conflito;
- um agente recebe no máximo uma referência para cada ocorrência.

### Ordem dos conflitos

A V1 percorre os caminhos antes dos tempos. Por isso, a ordem em que os conflitos são descobertos não garante que a lista de cada agente esteja ordenada por timestep.

Se o reparo local depender dessa propriedade, as listas precisarão ser ordenadas ao final ou armazenadas desde o início numa estrutura ordenada.

### Agente parado no destino

Não adicionar o conflito à lista do agente que já terminou é coerente com o comportamento do reparo local para `time > arrivalTime`: o caminho terminado não possui uma célula explícita nesse timestep, e o agente invasor é quem pode ser reparado.

Mesmo assim, a ocorrência global precisa registrar o agente parado como participante. Caso contrário, seu contador começaria em `1` e indicaria incorretamente que não existe conflito.

O caso `time == arrivalTime` deve ser tratado separadamente, porque o dono do destino ainda ocupa explicitamente a última posição do caminho e pode aparecer no mecanismo normal de ocupação de vértice.

### Atualização incremental durante o reparo

O mapa auxiliar não deve ser atualizado apenas decrementando a chave do conflito reparado. Um novo caminho pode:

- remover outros conflitos existentes no sufixo;
- continuar participando do mesmo conflito;
- criar conflitos em novos vértices ou timesteps;
- alterar seu tempo de chegada ao destino;
- alterar o makespan relevante para ocupações permanentes.

Uma atualização incremental correta precisa ter a proveniência de cada reserva e atualizar todas as ocorrências afetadas. Como alternativa mais simples e segura, os índices de conflito podem ser reconstruídos depois de cada reparo aceito.

### Chaves de tempo e célula

Usar o endereço de um objeto `CellTime` como chave não funciona. Dois objetos diferentes com os mesmos valores possuem endereços diferentes e, portanto, seriam considerados chaves distintas.

O struct deve ser usado por valor, com igualdade e hash próprios:

```cpp
struct CellTime {
    Cell* cell;
    int time;
};
```

O mesmo princípio vale para a chave de aresta e timestep. Uma chave de aresta também precisa preservar ou representar explicitamente sua direção.

## Estratégia corrigida para uma versão baseada na V1

Uma implementação baseada na V1 pode se aproximar de `O(total de células explícitas + conflitos produzidos)`, mas precisa incluir os seguintes elementos:

1. Pré-computar os destinos com o índice do proprietário e seu tempo de chegada.
2. Usar chaves estruturadas com `Cell*` e timestep, evitando strings.
3. Agregar todos os ocupantes explícitos de cada `(Cell*, time)` antes de produzir conflitos.
4. Considerar a ocupação virtual de um destino somente quando `arrivalTime < time`, pois no instante de chegada o agente ainda aparece explicitamente no caminho.
5. Registrar os participantes reais de cada ocorrência, em vez de manter somente um contador.
6. Para arestas, manter separadamente os participantes de cada direção e ignorar movimentos degenerados.
7. Produzir no máximo uma referência por agente e por ocorrência.
8. Ordenar os conflitos de cada agente por timestep antes que sejam usados pelo reparo local.
9. Reconstruir os índices depois de um reparo ou implementar uma atualização incremental completa, com proveniência por agente.
10. Formalizar se origens e destinos repetidos são proibidos e garantir essa validação antes de todas as chamadas de `getCollision` que dependam da restrição.

## Conclusão

A pré-computação dos destinos e a organização dos conflitos por agente são direções promissoras para reduzir o custo do reparo local. Entretanto, a combinação atual de mapa de destinos, flag booleana e contadores escalares não preserva todas as informações necessárias.

Antes de implementar a nova V1, é necessário decidir formalmente pela mudança do contrato de conflitos e representar explicitamente os participantes de vértices e as duas direções das arestas. Sem essas correções, a otimização pode produzir conflitos duplicados, perder participantes ou manter contadores incorretos depois dos reparos.
