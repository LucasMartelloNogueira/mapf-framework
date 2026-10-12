Entrada:
* agente
* caminho do agente (com conflitos)
* vetor de conflitos

Saída:
* novo caminho sem conflito foi encontrado: booleano
* novo caminho sem conflito (novo caminho sem conflito foi encontrado é verdadeiro, senão caminho vazio)

o objetivo é guardar os vértices que tem potenciais conflitos é o sufixo por vértice analisar por vértice, não por agente. Suponha que todas as seguintes condições foram atendidas:

* Todos os caminhos usados para construir o grafo continuam congelados.
* O grafo foi atualizado depois de cada reparo.
* Os objetivos ocupados indefinidamente estão representados.
* A estrutura considera todas as regras de conflito adotadas.
* Reparos paralelos não introduziram novos caminhos. (não vamos usar paralelismo para reparo local inicialmente)

Ao invés de buscar agentes candidatos que passam por vértices no sufixo, vamos analisar os vértices do sufixo e os safe intervals deles. Para isso precisamos analisar o vértice atual que estamos (qualquer vértice do início do sufixo até o penúltimo vértice do sufixo) e o próximo vértice do sufixo (que pode ser qualquer vértice entre o segundo vértice do sufixo até o último vértice do sufixo). Vamos considerar que cada um desses vértice se encaixam em dois casos:

1) o vértice não tem potencial conflito
2) o vértice tem potencial conflito

Como isso, temos 4 casos:

1) vértice atual não tem potencial conflito e próximo vértice não tem potencial conflito
2) vértice atual tem potencial conflito e próximo vértice não tem potencial conflito
3) vértice atual não tem potencial conflito e próximo vértice tem potencial conflito
4) vértice atual tem potencial conflito e próximo vértice tem potencial conflito

veja se o tratamento para cada um dos casos faz sentido

1) Não precisa checar nada, só fazer a transição para do vértice atual para o próximo vértice
2) Se chegamos até esse vértice no tempo t, isso significa que não há conflito no tempo t (mesmo podendo haver outros conflitos em um tempo diferente de t). E como não há potenciais conflitos no próximo vértice, podemos só fazer a transição do vértice atual para o próximo
3) Como só o próximo vértice tem potencial conflito, temos que usar a tabela de safe intervals e ver se a transição para o próximo vértice está em um safe interval, se não estiver, esperar o próximo safe interval.
4) Assumindo que o agente chegou no vértice atual no instante t, como não sabemos quando os agentes podem ter conflitos, o jeito seria realizar uma busca SIPP do agente do vértice atual até o próximo vértice e partindo do tempo t. Caso não haja solução, retorne false. No retorno false, o algoritmo anterior de reconstrução de caminhos local vai tentar uma janela maior, e repetir isso até conseguir fazer um reparo local. No pior caso, todo o caminho vai ser refeito

Observações importante:

* Usar vertex_agents (mapa no qual as keys são os vértices (Cell) e os valores são o conjunto de agentes que passam por aquele vértice) para cada vértice do sufixo do caminho para ver o vértice tem conflito ou não: Se só existe um agente por conjunto do vértice, então aquele vértice não tem potencial conflito, caso o tamanho do conjunto seja maior ou igual a 2, então aquele vértice tem potencial conflito
* também considerar os blockedEdgeArrivals de SafeIntervalTable para evitar conflitos de aresta