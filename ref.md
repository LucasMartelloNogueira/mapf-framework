Ideias: como armazenar os conflitos do MAPF no código?

Opção 1: 1 conflito para vários agentes

struct CellConflict {
    Cell* cell;
    int time;
    set[int] agents;
}

vantagens:
- gasta menos memoria
 - se achar um conflito, cria esse struct com dois agentes no set
 - se um outro agente também participar do conflito: adiciona ele no set

desvantagens:
- no reparo local, teria que percorrer por todos os conflitos para pegar o conflito de todos os agentes, o que pode ser bem custoso


Opção 2: 1 conflito por agente

dado o struct abaixo

struct CellConflict {
    Cell* cell;
    int time;
}

teriamos o hashmap onde cada agente armazena seus conflitos:

conflitos: {
    agente_1: {cellConflict1, cellConflict2, ...},
    agente_2: {cellConflict1, cellConflict3, ...},
    ...
}

vantagens:
- mais eficiente no reparo local, pois conserta o conflito e avalia o sufixo até o próximo conflito (os conflitos teriam que estar ordenados a partir do tempo em ordem crescente)
- no caso que um agente está parado no destino, só um conflito é gerado (para o agente que tenta passar pelo agente parado) e não dois conflitos
- no caso de 4 agentes estão em conflito em uma célula, como é um agente por célula, vou estar economizando memoria pois vão ser 4 conflitos, ao invés de 6 conflitos (combinação 4,2)

desvantagens
- gasta mais memória (em tese), pois no conflito mais comum, conflito de 1 célula para 2 agentes, vão ser gerados dois conflitos ao invés de 1


Estou pensando que deveria ter uma outra estrutura de dados, como um hashmap onde 
- as chaves são {t}-{x}-{y}: tempo t em que a celula de coordenadas x, y, teve conflitos
- os valores são quantos agentes tem conflito naquela célula

Assim no reparo local, um problema é que quando resolvemos o conflito para um agente, o outro agente pode já não ter um conflito.
Ex: se temos um conflito entre dois agentes: 1 e 2, no tempo 3, na célula x=10, y=20
- inicialmente nosso hashmap seria {3-10-20: 2}
- ao resolver o conflito do agente 1, via reparo local, poderiamos fazer hashmap[3-10-20] -= 1
- dessa forma, antes de resolver o conflito o agente 2, iriamos primeiro verificar o hashmap[3-10-20], caso o valor seja 1, isso significa que os outros conflitos já foram resolvidos e não existe mais aquele conflito para o agente 2

O mesmo mecanismo poderia servir para conflitos de arestas:
- 1 conflito por agente, ou seja, conflitos de arestas geram 2 conflitos
- usando a mesma ideia de hashmap que armazena quantidade de agentes por conflito (só teria que ajustar a chave para ser {t}-{x1}-{y1}-{x2}-{y2})
- se o hashmap de conflitos, para uma chave for 1, podemos ignorar o conflito de aresta para o agente


Ao invés de usar chaves de tempos e de coordenadas de células, se criar um struct CellTime com a Cell* e o tempo e utilizar o endereço desse struct for mais barato, podemos utilizar isso



==========================

Depois de perguntar ao codex sobre a sugestão acima e sobre como implementar getCollisionV1 de maneira válida, algumas problemas surgiram e essa foi minha tentativa de solucionar eles

aqui vou tentar resolver todos os problemas observados:

1) Autoconflito: podemos armazenar o indice do agente no caminho no valor do map, para ignorar que o final do caminho do próprio agente seja considerado um conflito. Então o valor do mao deve ser um struct com o indice do caminho e o tempo de chegada t

2) Conflito duplicado: podemos usar uma flag booleana se detectarmos um conflito por goalVertexLookup, que irá virar true quando encontramos um conflito por goalVertexLookup. Quando essa flag for true, não vamos verificar o conflito de vertexLookup.

3) Um inteiro por célula é insuficiente: Além de já usarmos o índice do caminho para identificar o agente, não iremos considerar nesse caso do MAPF que agentes podem compartilhar a mesma origem ou destino, pois isso isso tornaria a instancia inválida e não vamos considerar que ela aconteça.

4) A V1 não preserva multiplicidade: vamos mudar o formato como armazenamos os conflitos para ajudar na velocidade do reparo local, ler ref.md para mais detalhes, vamos usar a opção 2, com um conflito por agente e o hashmap auxiliar que armazena a quantidade de agentes por conflito

5) O mesmo problema existe nas arestas: ref.md, com a opção 2, com um conflito por agente e o hashmap auxiliar que armazena a quantidade de agentes por conflito, também cobre o caso de conflitos de arestas

6) Esperas podem virar falsos conflitos de aresta: poderiamos usar a ideia de getCollision e levar ela para GetCollisionV1, e verificando somente se os agentes tenham se movido

7) Faltam conflitos posteriores ao fim dos dois caminhos: novamente, nesta versão do MAPF, nenhum agente pode compartilhar origens e destinos, então esse caso não vai acontecer

Passei novamente esses problemas para o codex e outras observações foram anotadas em getCollisions.md
