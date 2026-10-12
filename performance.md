**O principal motivo é que o reparo local faz muitas alterações pequenas, mas copia e descarta todas as reservas em cada uma delas.** A economia obtida com buscas SIPP curtas acaba sendo consumida por esse trabalho.

Nos resultados fornecidos, ambos resolvem o mesmo cenário, com 100 agentes e caminhos iniciais idênticos:

| Solver | Tempo |
|---|---:|
| LocalPathRepairIterativeSolver (results/test\_1791491601911/test\_1791491601911\_stats.csv) | 3,243 s |
| FullPathRepairIterativeSolver (results/test\_1791488789109/test\_1791488789109\_stats.csv) | 1,327 s |

O local levou **2,44 vezes mais tempo**.

Para aprofundar a análise, executei uma cópia temporária instrumentada dos dois solvers, usando a mesma instância. Ela reproduziu os custos finais e makespans dos arquivos apresentados. Os números abaixo são dessa execução adicional; os tempos são indicativos, pois incluem instrumentação.

| Trabalho realizado | Local | Completo |
|---|---:|---:|
| Iterações de reparo | **192** | **28 grupos** |
| Cópias completas das reservas | 192 | 28 |
| Buscas SIPP | 192 pontes + 1 fallback | 57 caminhos completos |
| Novos eventos introduzidos entre atualizações¹ | **308** | **0** |
| Tempo copiando e substituindo reservas | **2,897 s** | **0,365 s** |
| Tempo nas buscas SIPP | 0,004 s | 0,387 s |
| Tempo recalculando conflitos | 0,027 s | 0,404 s |

¹ Contagem acumulada de eventos ausentes no estado imediatamente anterior; não significa 308 conflitos simultâneos.

**1\. A cópia global das reservas é o maior gargalo confirmado.**

Em repairInitialPaths (src/mapf/solvers/local\_path\_repair\_solver\_common.cpp:497), cada reparo começa com:

```
PathReservationState preparedReservations = result.reservations;
```

Essa cópia inclui os mapas de ocupações, bloqueios de arestas e intervalos seguros. A tabela contém entradas para **65\.792 células**, incluindo obstáculos e células sem reservas.

Com 192 reparos, somente essa tabela acumula **12,6 milhões de entradas copiadas**, além das demais estruturas.

O flamegraph (profiling/20261008T203315Z-211977/flamegraph.svg) atribui aproximadamente **45,15%** à construção/cópia de `PathReservationState`. Na instrumentação, copiar e substituir as reservas consumiu aproximadamente **79% do tempo do local**.

A atribuição final com `std::move` também custa tempo: ela transfere as novas estruturas, mas precisa liberar os nós das reservas antigas. Isso explica a presença forte de `malloc`, `free` e desalocação de tabelas hash no perfil.

**2\. O reparo local gera muito mais trabalho posterior.**

O local resolve a ponte e reaproveita o sufixo. Quando a ponte muda o tempo de chegada à reconexão, esse sufixo passa a encontrar outras reservas em instantes diferentes.

Nesta execução:

- 32 reparos aumentaram a quantidade de eventos;
- 28 mantiveram a quantidade;
- o agente 55 foi reparado **18 vezes**;
- os agentes 9 e 39 foram reparados **16 vezes cada**.

Já o reparo completo (src/mapf/solvers/full\_path\_repair\_iterative\_solver.cpp:174) remove as reservas de todo o grupo e planeja os caminhos completos, respeitando os demais agentes. Nesta execução, seus 57 participantes foram replanejados uma vez cada, em 28 grupos.

Portanto, o local paga o custo global das reservas **6,86 vezes mais frequentemente**.

**3\. A detecção de configurações repetidas acrescenta outro custo global.**

configurationFingerprint (src/mapf/solvers/local\_path\_repair\_solver\_common.cpp:197) percorre **todos os caminhos** e serializa todas as coordenadas usando `ostringstream`, mesmo quando apenas um caminho mudou.

Foram:

- 193 chamadas;
- aproximadamente **24,4 MB de texto produzido**;
- 0,367 s na execução instrumentada;
- **8,83% no flamegraph**.

Nenhuma configuração repetida foi rejeitada nesse caso. A proteção continua tendo utilidade, mas sua implementação cobra um custo significativo por tentativa.

**4\. As buscas curtas e a atualização incremental estão economizando trabalho.**

As buscas SIPP locais consumiram aproximadamente 4 ms, e houve apenas um fallback. Logo, expandir âncoras ou executar muitos fallbacks não explica esse resultado.

A atualização incremental também foi bem mais barata que recalcular os conflitos globalmente. A vantagem dessas duas operações foi consumida pelas cópias de reservas e pela assinatura das configurações.

Além disso, a manutenção das reservas ainda remove e reinsere **o caminho inteiro do agente**, mesmo quando a ponte altera poucas células.

Eu priorizaria estas melhorias:

1. **Eliminar a cópia global por reparo**, registrando apenas as alterações necessárias e permitindo restaurá-las quando uma tentativa falhar.
2. **Calcular a assinatura incrementalmente por agente**, preservando a proteção contra ciclos e tratando possíveis colisões de hash.
3. **Limitar reparos sucessivos pouco produtivos**, antecipando o replanejamento completo quando um agente acumular reparos ou continuar criando conflitos.

A evidência mais forte aponta para as duas primeiras: o perfil mostra que o custo atual está concentrado na administração das estruturas globais.