# Hash global de 128 bits formado pelos hashes dos agentes

Sim. A ideia é manter um hash por caminho e combinar esses hashes de modo que seja possível retirar a contribuição antiga de um agente e inserir a nova sem recalcular os demais.

## Por que as esperas precisam ser diferenciadas?

Em MAPF, o caminho não representa apenas uma sequência espacial: o índice de cada célula representa o tempo.

Considere:

```text
P1 = [A, B]
P2 = [A, A, B]
```

Em `P2`, o agente espera em `A` durante um instante.

| Tempo | P1 | P2 |
|---:|---|---|
| 0 | A | A |
| 1 | B | A |
| 2 | permanece em B | B |

Esses caminhos produzem reservas diferentes:

- `P1` ocupa `B` a partir de `t=1`;
- `P2` ocupa `A` em `t=1` e chega a `B` em `t=2`;
- as arestas são atravessadas em instantes diferentes;
- os conflitos com outros agentes podem ser completamente diferentes.

Por isso, o hash precisa considerar cada ocorrência de célula com sua posição temporal:

```text
(cell=A, time=0)
(cell=A, time=1)
(cell=B, time=2)
```

Não é necessário criar um símbolo especial para “espera”. Basta que o hash seja sensível à ordem, ao tempo e ao tamanho do caminho.

Por exemplo, uma entrada conceitual adequada seria:

```text
hashPath([
    (0, A),
    (1, A),
    (2, B)
], tamanho=3)
```

Enquanto o outro caminho seria:

```text
hashPath([
    (0, A),
    (1, B)
], tamanho=2)
```

### O que não fazer

Não se deve calcular o hash do caminho fazendo simplesmente:

```cpp
hash ^= hash(cell);
```

Isso perde a ordem e ainda permite cancelamentos:

```text
hash(A) XOR hash(A) = 0
```

Assim, uma espera poderia desaparecer do hash.

Uma forma adequada é misturar também o instante:

```text
pathHash = XOR, para cada t:
    Mix128(t, cellId[path[t]])

pathHash ^= Mix128("path-length", path.size())
```

Ou usar um hash sequencial:

```text
hash = Mix128(hash, time, cellId)
```

O identificador da célula deve ser estável, por exemplo:

```text
cellId = y * gridWidth + x
```

Usar o endereço `Cell*` funcionaria dentro da mesma execução, mas produziria valores diferentes entre processos por causa dos endereços de memória.

## Como o hash global funciona

Suponha que cada agente possua um hash de caminho:

```text
pathHash[0]
pathHash[1]
pathHash[2]
```

Antes de juntar esses hashes, é necessário vinculá-los à identidade do agente:

```text
contribution[i] = Mix128(agentId=i, pathHash[i])
```

Então o hash global é:

```text
configHash =
    contribution[0]
    XOR contribution[1]
    XOR contribution[2]
```

A identidade do agente é essencial. Sem ela, trocar os caminhos dos agentes 0 e 1 produziria o mesmo resultado, porque XOR não depende da ordem.

## Por que XOR permite retirar uma contribuição

XOR possui estas propriedades:

```text
X XOR X = 0
X XOR 0 = X
```

Suponha:

```text
G = A XOR B_antigo XOR C
```

Queremos substituir `B_antigo` por `B_novo`.

Primeiro fazemos:

```text
G XOR B_antigo
```

Expandindo:

```text
A XOR B_antigo XOR C XOR B_antigo
```

Como:

```text
B_antigo XOR B_antigo = 0
```

resta:

```text
A XOR C
```

Depois acrescentamos a nova contribuição:

```text
G' = G XOR B_antigo XOR B_novo
```

Resultado:

```text
G' = A XOR C XOR B_novo
```

Os hashes dos agentes A e C não precisam ser recalculados.

## Exemplo numérico simplificado

Vou usar hashes de 8 bits apenas para facilitar a visualização. Na implementação real seriam 128 bits.

Considere:

```text
contribution[agente 0]        = 00110110  // 0x36
contribution[agente 1 antiga] = 10101100  // 0xAC
contribution[agente 2]        = 01010001  // 0x51
```

O hash global atual é:

```text
00110110 XOR 10101100 = 10011010
10011010 XOR 01010001 = 11001011
```

Portanto:

```text
configHash = 11001011  // 0xCB
```

Agora o agente 1 recebe um novo caminho:

```text
contribution[agente 1 nova] = 11110000  // 0xF0
```

Aplicamos:

```text
configHash'
    = configHash
    XOR contribution[agente 1 antiga]
    XOR contribution[agente 1 nova]
```

Numericamente:

```text
11001011 XOR 10101100 = 01100111
01100111 XOR 11110000 = 10010111
```

O novo hash global é:

```text
configHash' = 10010111  // 0x97
```

Se recalculássemos diretamente:

```text
00110110 XOR 11110000 XOR 01010001
= 10010111
```

Obtemos exatamente o mesmo valor.

## Estrutura do algoritmo

Conceitualmente:

```cpp
Hash128 configHash{};
std::vector<Hash128> pathHashes(agents.size());

for (std::size_t i = 0; i < agents.size(); ++i) {
    pathHashes[i] = hashPath(paths[i]);
    configHash ^= agentContribution(i, pathHashes[i]);
}

std::unordered_set<Hash128, Hash128Hasher> visited;
visited.insert(configHash);
```

Ao gerar um candidato para o agente `i`:

```cpp
Hash128 newPathHash = hashPath(newPath);

Hash128 candidateHash =
    configHash ^
    agentContribution(i, pathHashes[i]) ^
    agentContribution(i, newPathHash);
```

A verificação acontece antes da confirmação:

```cpp
if (visited.contains(candidateHash)) {
    // A configuração provavelmente já foi visitada.
    // Tentar outra âncora ou o fallback.
}
```

Quando o candidato for aceito:

```cpp
configHash = candidateHash;
pathHashes[i] = newPathHash;
visited.insert(configHash);
```

Se o candidato for rejeitado, nada precisa ser restaurado porque `configHash` e `pathHashes[i]` ainda não foram modificados.

## Uma forma de construir a contribuição

Conceitualmente:

```cpp
Hash128 agentContribution(
    std::size_t agentIndex,
    Hash128 pathHash
) {
    return mix128(agentIndex, pathHash);
}
```

É importante não fazer apenas:

```cpp
return pathHash;
```

Senão estas configurações poderiam ter o mesmo hash global:

```text
Configuração 1:
agente 0 → caminho X
agente 1 → caminho Y

Configuração 2:
agente 0 → caminho Y
agente 1 → caminho X
```

Como:

```text
hash(X) XOR hash(Y) == hash(Y) XOR hash(X)
```

Misturando o índice:

```text
Mix(0, hash(X)) XOR Mix(1, hash(Y))
```

fica diferente de:

```text
Mix(0, hash(Y)) XOR Mix(1, hash(X))
```

## Resumo dos elementos obrigatórios

O hash do caminho deve considerar:

- a célula;
- a ordem ou instante;
- repetições de células;
- o tamanho do caminho.

A contribuição global deve considerar:

- o índice/ID do agente;
- o hash de seu caminho.

A atualização fica barata porque somente o caminho modificado é novamente processado:

```text
custo anterior: O(soma do tamanho de todos os caminhos)
custo incremental: O(tamanho do caminho do agente alterado)
```

E cada configuração visitada exige guardar somente 128 bits, além do overhead do `unordered_set`.
