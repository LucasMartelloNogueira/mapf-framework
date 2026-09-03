Imagine o seguinte cenário

s -> ... -> a -> x -> b -> ... -> g

onde:

s: vértice de origem do agente
g: vértice de destino do agente
x: vértice onde o agente tem conflito


Ao invés de reparar todo o caminho, podemos apenas não utilizar o vértice com conflito e usar outro caminho.
Dessa forma, temos que somente recalcular o caminhos entre os vértices A e B

Problemas

Temos dois cenários:

1) chegamos no vértice B no mesmo instante de tempo
2) chegamos no vértice B em outro instante de tempo 

Caso aconteça o cenário 1: 
* assumindo que não temos mais nenhum conflito do vértice B até G. Podemos reaproveitar calcular um novo caminho
  dos vértices A até B e reaproveitar o caminho de B até G.

caso aconteça o cenário 2, temos dois fatores:

Fator 1: Dado que o agente chegou no vértice B no instante t no caminho original, o agente chegou antes de t em B depois de ter calculado o outro caminho? (sim ou não)
Fator 2: existe 1 ou mais agentes nos quais seus caminhos passam por algum vértice de B até G (sim ou não)

dados que esses dois fatores podem ser respondidos com sim ou não, temos uma combinação de 4 cenários:

cenário 1: Fator 1 - sim / Fator 2 - sim
    fator 1: agente chegou no vértice B em algum instante antes de t
    fator 2: existe 1 ou mais agentes nos quais seus caminhos passam por algum vértice de B até G
 
* originalmente, quando o agente chega em B no tempo t, não há conflitos de B até G
* o fato do agente chegar antes e não esperar pode significar que terá conflitos com os outros agente que passam pelo seu caminho
* Supondo que o agente chega k unidades de tempo antes (chegando no tempo t-k), o agente pode esperar k timesteps e depois seguir seu caminho.
  Isso seria a opção mais simples supondo que o caminho de B até G é válido quando o agente sai de B em t
* alternativamente, o agente caso o agente chegue antes, ele pode seguir sua viagem sem esperar. A única coisa que ele teria que fazer é calcular o safe intervals
  para todos os vértices no restante do caminho para garantir que o movimentos são válidos

==================

cenário 2: Fator 1 - sim / Fator 2 - não
    fator 1: agente chegou no vértice B em algum instante antes de t
    fator 2: nenhum outro agente passa, em seus caminhos, por algum vértice do caminho de B até G

* com o fator 2 sendo "não", não existe a possibilidade de novos conflitos, independente do tempo que o agente chegou em B
* sendo assim, o agente pode seguir o caminho de B até G, e como chegou antes em B, um caminho mais curto foi encontrado

==================

cenário 3: Fator 1 - não / Fator 2 - sim
    fator 1: agente chegou no vértice B em algum instante depois de t
    fator 2: existe 1 ou mais agentes nos quais seus caminhos passam por algum vértice de B até G

* o cenário mais complexo
* como o fator 2 simboliza a possibilidade de conflitos e o agente chega depois em  B, não podemos assumir que o caminho de B até G continua sendo válido,
  pois novos conflitos podem ser concretizados com a chegada do agente em um tempo depois de t
* opção simples: considerando que o agente chegou no tempo t+k, refazer o caminho de B até G considerando o nó atual como sendo B no tempo t+k
* outra opção: identificar os vértices que tem potenciais conflitos de B até G. Quando o agente chega em B,
  para todos vértice de B até G, caso o vértice tenha um potencial conflito, calcular o safe interval do vértice, se estiver dentro do safe interval
  original, permite o movimento, caso contrário, tem que procurar outro safe interval do vértice em questão. Se nenhuma transição for válida,
  tem que fazer uma busca SIPP considerando o vértice Vn até Vn+1 e considerar o nó inicial sendo Vn no tempo tn


==================

cenário 4: Fator 1 - não / Fator 2 - não
    fator 1: agente chegou no vértice B em algum instante depois de t
    fator 2: nenhum outro agente passa, em seus caminhos, por algum vértice do caminho de B até G

* o fator 2 implica que não há potenciais conflitos do caminho de B até G
* isso significa que o agente pode seguir caminho sem se preocupar com conflitos
* se o agente demorou mais k timesteps até chegar em B, o caminho final terá um custo adicional de k