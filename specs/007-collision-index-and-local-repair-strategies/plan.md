Created: 2026-09-15T21:26:04-03:00

Author: Lucas Martello Nogueira (local user: lucas; repository Git identity)

Last updated: 2026-09-16T19:40:34-03:00

AI model: GPT-6 (Codex)

# Plan: Collision Indexes and Local Repair Strategies

## Status and scope

This document plans the work requested in `task.md`. It does not implement the feature. The requested deliverable is a plan in English, saved according to `.github/ai-instructions.md`. The original request and task text are preserved below.

The implementation will replace pairwise collision output with one repair record per participating explicit path, validate unique starts and goals at construction, introduce two repair strategies, and limit local suffix processing to the next conflict of the active agent.

The baseline is the current working tree, including the user's unfinished `getCollisionV1` draft and updated MAPF constraints. Use that draft and its proposed corrections as the basis for updating `getCollision` directly, then remove `getCollisionV1`. The final API and implementation must contain only one collision detector, named `getCollision`. The unrelated deletion of `tutorial_perf_flamegraph_mapf.md` is outside this feature.

## prompt

### User request

```text
leia o arquivo task.md e faça o que se pede
```

### Task definition from task.md

```text
Leia os arquivos .github/ai-instructions.md, ref.md e getCollisions.md e monte um plano para as seguintes funcionalidades:

* implementar a função getCollisionV1 da maneira sugerida para que ela se torne mais rápida, resolvendo os problemas que foram documentados
* mudar a forma de armazenar conflitos no código para um conflito por agente (exemplo: 1 conflito com dois agente == 2 conflitos, um para cada agente / 1 conflito de 3 agentes == 3 conflitos, um para cada agente) para agilizar o reparo local

A seguir estão algumas sugestões depois da leitura de getCollisions.md

1) Autoconflito: deve-se fazer o seguinte em getCollisionV1

* pré-popular mapa de destinos (goalVertexLookup) no qual as chaves são Cell* e os valores são um struct que tem os campos: i int e time int. O i serve para identificar o agente (que é o index de paths) e t é o instante de tempo que o agente i chega em seu destino
* declarar um int i = 0 externo, antes do primeiro for loop de paths e incrementar a cada iteração
* no segundo for loop (que itera as células de um caminho) é preciso botar uma condição que verifica o i para excluir o auto conflito da seguinte forma: if (goalVertexLookupItem != goalVertexLookup.end() &&  goalVertexItem->second.i != i && goalVertexLookupItem->second.arrivalTime <= t)

Com isso, conseguimos evitar o autoconflito

3) Não admitir origens ou destinos repetidos: deve-se fazer o seguinte

* já foi atualizado .github/mapf.md para que não seja considerado instâncias com origens e destinos repetidos
* criar uma função em src/mapf/core/instance.cpp chamada isValidInstance, na qual se cria dois mapas, um de origem e um de destino, eles devem ser usados para verificar que nenhum par de agentes tem dois destinos repetidos. Para verificar a duplicidade na origem, para cada agente, veja a sua célual inicial se encontra no mapa (use as coordenadas x e y como chaves e o id do agente como valor). Se não houver uma entrada nesse mapa, continue, se houve, retorne falso. Caso chegue no final e não tenha ocorrido nenhum erro, retorne true. Faça o mesmo procedimento de forma analoga para a coordenada de destino, mas faça a verificação da origem e destino de um agente na mesma iteração.
* bote essa validação dentro do construtor e de um throw std::invalid_argument caso isso aconteça, uma instancia invalida não deveria existir
* mude os testes que contabilizam esses casos de origem e/ou destinos compartilhados para que isso não aconteça, o teste deve verificar o retorno do erro no construtor em caso de uma instância invalida
* retirar em src/mapf/solvers/local_path_repair_solver_common.cpp a verificação hasSharedStarts e hasDuplicateGoals

4) Seguir recomendação de getCollisions.md. Fazer as implementações necessárias alterar um conflito por agente. Em src/mapf/solvers/local_path_repair_solver_common.cpp na função repairInitialPaths adotar um swtich case da estratégia usada para reparo de conflitos, as duas estratégias inicias são RESOLVE_BY_AGENT e RESOLVE_BY_TIME. Ambas as estratégias são escolhidas via cli no argumento da flag opcional localRepairStrategy. Se essa flag não for selecionada, usar RESOLVE_BY_AGENT como default. As duas estratégias consistem no seguinte:

RESOLVE_BY_AGENT: utiliza o índice dos agentes como prioridade, primeiro resolve todos os conflitos do agente i, depois todos os conflitos do i+1 e assim por diante
RESOLVE_BY_TIME: estratégia atual (mas precisa de ajustes) que resolve primeiro os conflitos que aconteceram primeiro.

Além disso, se atente nos seguinte ajustes

* mapeie e faça as alterações necessárias para se adequar ao novo modelo de um conflito por agente
* sempre que fizer o reparo local, não avalie todo o sufixo, apenas o trecho do conflito atual até o próximo conflito, utilize a lista de conflitos dos agente para determinar esse trecho. 

5) Seguir recomendação de getCollisions.md

6) Seguir recomendação de getCollisions.md

7) Seguir recomendação de getCollisions.md
```

## Sources and precedence

Read together:

- [Project instructions](../../.github/ai-instructions.md).
- [MAPF assumptions](../../.github/mapf.md).
- [Representation proposals](../../ref.md).
- [Correctness review](../../getCollisions.md).
- [Collision reporting plan](../003-solution-collision-reporting/plan.md) and [specification](../003-solution-collision-reporting/spec.md).
- [Local repair plan](../004-local-path-repair-parallel-solver/plan.md) and [specification](../004-local-path-repair-parallel-solver/spec.md).
- [CLI and iterative solver plan](../005-cli-results-and-iterative-local-repair/plan.md) and [specification](../005-cli-results-and-iterative-local-repair/spec.md).
- [Profiling plan](../006-perf-flamegraph-profiling/plan.md).
- [Local repair scenarios](../../docs/local_path_repair.md), [suffix procedure](../../docs/path_sufix_repair_algorithm.md), [implemented repair behavior](../../docs/local_path_repair_parallel_solver.md), and [CLI/result contracts](../../docs/experiment_cli_and_results.md).

The new feature intentionally supersedes the old pairwise conflict representation, solver-time rejection of duplicate endpoints, and whole-suffix processing during a local attempt. Stay-at-target semantics, sequential repair, aligned path slots, transactional commits, full-path fallback, and partial results remain applicable. Historical plans and their recorded prompts should remain intact; the future specification must identify these replacements explicitly. The numbered entries in `adjustments` override conflicting instructions in the preserved original prompt, including its requests to implement `getCollisionV1` and modify tests.

## Existing code and baseline findings

| Area | Current behavior | Required change |
| --- | --- | --- |
| `src/mapf/utils.cpp` | `getCollision` compares every pair at every timestep, including virtual goal occupancy. | Aggregate occupancy using structured keys, then emit agent-indexed records. |
| `getCollisionV1` draft | Uses string keys, stores one occupant, inserts goals after traversing each path, and can count waits as swaps. | Apply the corrected approach directly in `getCollision`: prepopulate goals, retain ownership and multiplicity, aggregate before emission, and skip waits. Remove the V1 draft afterward. |
| Conflict headers | `SolutionConflicts` holds two flat vectors without agent ownership; records contain reference/const members. | Introduce event membership maps and one sorted list per path; make records assignable for sorting. |
| `Instance` | Manual construction checks IDs and free cells; file loading checks scenario data. Neither enforces distinct starts/goals. | Add `isValidInstance` to both construction paths. |
| Common repair engine | Agent-major outer loop, earliest explicit conflict within each agent. Repeated global scans infer ownership from cells/times. | Direct selection from agent lists; real global time ordering as an alternative. |
| Local candidate handling | `buildLocalCandidate`, `appendScenarioTwoPointThreeSuffix`, and `earliestConflictWithFrozenPaths` examine the whole suffix/path. | Bound candidate processing; reuse the shared collision index for progress and consistency checks. |
| Reservation handling | Rebuilds all reservations, sometimes repeatedly for the same candidate/state. | Keep safe rebuild semantics and avoid duplicate rebuilds for a committed revision. |
| Experiment output | `normalizeConflicts` independently repeats pairwise detection, and callers plus the writer normalize twice. | Project the shared event index into grouped CSV rows and normalize once in the writer. |

The prompt describes `RESOLVE_BY_TIME` as the current strategy. Code inspection shows that the current strategy is agent-major: `repairInitialPaths` loops over `activeIndex`, then chooses that agent's earliest conflict. The new default therefore preserves that priority principle. `RESOLVE_BY_TIME` must compare conflicts across agents.

A read-only baseline check was executed:

```bash
c++ -std=c++20 -Iinclude -fsyntax-only src/mapf/utils.cpp
```

It failed in the existing V1 draft: missing `<unordered_map>`, undefined `GoalOccupancy`, and undefined `positionKey`, `positionTimeKey`, and `edgeTimeKey`. The draft is also absent from `include/mapf/utils.hpp`. Existing build artifacts do not establish that this working tree builds. No passing baseline test suite or performance result is claimed.

## Libraries and documentation researched

Keep C++20 and the existing standard-library/CMake stack. No new runtime dependency is necessary.

| Facility | Use and reason | Primary documentation |
| --- | --- | --- |
| `std::unordered_map`, `std::unordered_set`, `std::hash` | Value-keyed occupancy and idempotent participant membership; reserve capacity for known input sizes. Equivalent keys must hash equally. Iteration order is not scheduling order. | [C++ unordered container requirements](https://eel.is/c++draft/unord.req) |
| `std::vector`, `std::variant`, `std::sort` | Dense path-indexed lists containing vertex or edge records; sort with an explicit deterministic comparator. Sorting contributes to total complexity. | [C++ sorting requirements](https://eel.is/c++draft/alg.sorting) |
| Existing SIPP and reservation tables | Reuse time-offset bridges, safe intervals, reverse-edge blocking, and permanent-goal fallback. | Local SIPP headers/implementation and specification 004 |
| CMake/CTest and existing profiling scripts | Run existing correctness checks and compare optimized builds using the existing workflow. | [CTest manual](https://cmake.org/cmake/help/latest/manual/ctest.1.html) |

[Abseil containers](https://abseil.io/docs/cpp/guides/container) were considered as an alternative hash-table implementation. They are not selected: the algorithm and ownership model should be corrected and measured before introducing another dependency or changing container lifetime assumptions.

The documentation was consulted on 2026-09-15. Use only facilities available in C++20; examples in current online documentation may describe newer language versions.

## Domain and API contracts

1. `pathIndex` is the slot in `paths` and `Instance::getAgents()`; it is not `Agent::id` or `scenarioId`. It controls repair priority. Convert to real IDs only at reporting boundaries.
2. Distinct agents must have distinct starts and distinct goals. These are independent constraints: sharing either one invalidates the instance. An agent may start at its own goal, and one agent's start may equal another agent's goal.
3. Every nonempty full path uses live cells from the same grid, begins at absolute time zero, and has arrival time `size() - 1`. Empty paths retain their vector slot, create no reservations, and still cause solver failure when a required path is missing.
4. At `time == arrivalTime`, the destination occupant is explicit and may receive a repair record. At `time > arrivalTime`, the occupant is virtual, remains a global event participant, and does not receive a repair record.
5. Vertex conflicts contain at least two distinct participants. Edge records represent swaps: a nondegenerate edge has participants in both directions at the same arrival time.
6. Same-direction simultaneous traversal is already invalid through its vertex occupancies; it does not create an additional swap record. Following and cycles without vertex/swap collisions remain allowed.
7. Grid cell pointers are identity keys within one live grid. Never use the address of a temporary key struct as identity. Do not mutate/move the grid while indexed paths are in use.
8. Since the standalone detector receives paths without an `Instance`, its single public entry point, `getCollision`, must also reject duplicate starts/goals. Validate them while precomputing endpoints; also reject null cell pointers and unrepresentable integer times/indexes before dereferencing/conversion. Endpoint checks must not be only debug assertions.
9. `getCollision` throws `std::invalid_argument` for invalid detector inputs. `validateSolution` remains a boolean collision predicate: return `false` for those invalid inputs, otherwise return `getCollision(paths).empty()`. Preserve the existing rule that this predicate alone does not establish that all required paths exist.

The restriction concerns actual full-path endpoints, not intermediate meetings. Validation inputs for intermediate collisions must have distinct destinations.

## Proposed conflict representation

Use two complementary views of the same occurrence: global membership and the repair records indexed by path. An occurrence shared by three explicit agents yields three repair records; global membership is stored once.

Write types explicitly throughout the proposed implementation and examples, even for long nested standard-library containers. Do not introduce type aliases with `using`, `typedef`, or equivalent shortcuts. Use `int`, `std::unordered_set<int>`, and `std::variant<CellConflict, EdgeConflict>` directly; the domain structs below remain concrete types.

Illustrative declarations (names and contracts to retain in the specification):

```cpp
struct CellTime {
    Cell* cell;
    int time;
    bool operator==(const CellTime&) const = default;
};

struct EdgeTime {
    Cell* first;   // canonical coordinate order
    Cell* second;
    int time;      // arrival time of the traversal
    bool operator==(const EdgeTime&) const = default;
};

struct CellConflict {
    Cell* cell;
    int time;
};

struct EdgeConflict {
    Cell* cell_1;  // canonical endpoint, not the active agent's departure
    Cell* cell_2;
    int time;
};

struct VertexEvent {
    std::unordered_set<int> participants;
};

struct EdgeEvent {
    std::unordered_set<int> forward;  // first -> second
    std::unordered_set<int> reverse;  // second -> first

    bool active() const {
        return !forward.empty() && !reverse.empty();
    }
};

struct SolutionConflicts {
    std::unordered_map<CellTime, VertexEvent, CellTimeHash> vertexEvents;
    std::unordered_map<EdgeTime, EdgeEvent, EdgeTimeHash> edgeEvents;
    std::vector<std::vector<std::variant<CellConflict, EdgeConflict>>> byAgent;

    bool empty() const {
        return vertexEvents.empty() && edgeEvents.empty();
    }
};
```

The public snapshot maps contain active events only. Temporary occupancy maps also contain singleton/nonconflicting occupancies until detection finishes. `byAgent.size()` always equals `paths.size()`, even with no collisions, so outer-vector emptiness is not a solution-validity check.

`CellTimeHash` and `EdgeTimeHash` hash every equality field by value, using `std::hash<Cell*>` and `std::hash<int>` and a shared unsigned hash-combine helper. Reuse the project's separation of key/hash declarations and implementations; do not reuse SIPP's interval index as a timestep. Canonicalize edges by `(x, y)`, not built-in ordering of unrelated pointers.

An agent's record refers to its global occurrence by its value key. It contains neither a raw pointer nor an iterator into another container. This makes copying/moving `initialConflicts` and `remainingConflicts` safe across snapshots. Destroy all selection cursors when replacing a snapshot.

Counts are derived from membership sizes, rather than maintained as independently decremented scalar counters. For swaps, a total count of three can mean no conflict if all remaining participants move in the same direction.

| Occupancy | Global participants | Repair records |
| --- | ---: | ---: |
| Two explicit agents at one vertex | 2 | 2 |
| Three explicit agents at one vertex | 3 | 3 |
| Four explicit agents at one vertex | 4 | 4, not six pairwise records |
| One visitor and one virtual destination owner | 2 | 1, for the visitor |
| Two visitors and one virtual destination owner | 3 | 2, one per visitor |
| One explicit arrival and one other explicit occupant | 2 | 2 |
| Three forward traversals and one reverse traversal | 4, in two sets | 4 |

Within each agent list, sort by `(time, kind, canonical coordinates)`, with vertex before edge for ties. Emit each `(pathIndex, event key)` once. This also defines deterministic ordering when hash iteration and input completion order vary.

## Instance validation

Add a private `bool Instance::isValidInstance() const` declaration to `include/mapf/core/instance.hpp` and its implementation to `src/mapf/core/instance.cpp`. Its new responsibility is endpoint uniqueness; retain the existing checks for free cells, IDs, dimensions, and file format.

Use a private coordinate key and hash, both based on `x` and `y`:

```cpp
bool Instance::isValidInstance() const {
    std::unordered_map<CoordinateKey, int, CoordinateKeyHash> startOwners;
    std::unordered_map<CoordinateKey, int, CoordinateKeyHash> goalOwners;
    startOwners.reserve(agents.size());
    goalOwners.reserve(agents.size());

    for (const Agent& agent : agents) {
        const CoordinateKey start{
            agent.startPosition.x, agent.startPosition.y
        };
        const CoordinateKey goal{
            agent.goalPosition.x, agent.goalPosition.y
        };
        if (!startOwners.emplace(start, agent.id).second) {
            return false;
        }
        if (!goalOwners.emplace(goal, agent.id).second) {
            return false;
        }
    }
    return true;
}
```

Call it from both constructors once the selected agent collection is available and before construction completes:

```cpp
if (!isValidInstance()) {
    throw std::invalid_argument(
        "Agent starts and agent goals must each be unique."
    );
}
```

Do not route this rejection through the current `require` helper, which throws `std::runtime_error`. The file constructor validates only the requested scenario prefix, consistent with its existing `numAgents` behavior.

After both constructors and the detector boundary are covered, remove `hasSharedStarts`, `hasDuplicateGoals`, and their branches from the common repair source. Keep missing-initial-path handling. Invalid CLI input now fails during construction with exit code 2 and no result bundle, including when continuation is enabled.

## getCollision algorithm

Implement the following passes directly in the existing `getCollision` function. The V1 draft is reference material for this implementation, not a second function to retain or expose.

### Endpoint pass

Prepopulate `std::unordered_map<Cell*, GoalOccupancy> goalVertexLookup`:

```cpp
struct GoalOccupancy {
    int i;
    int arrivalTime;
};
```

Validate unique starts and goals before relying on one owner per destination. Reserve map capacity from the number of paths. Count explicit cells/moves to size other maps without allocating by `agents * makespan`.

Use an external `int i = 0` for the path index, incrementing once for every vector slot, including an empty path. Do the same in the occupancy pass; never renumber after skipping empty paths.

### Occupancy pass

For every explicit `cell` at time `t`, insert `i` into the participant set for `CellTime{cell, t}`. Consult the prepopulated goal map:

```cpp
std::unordered_set<int>& occupants = vertexLookup[CellTime{cell, t}].participants;
occupants.insert(i);

const std::unordered_map<Cell*, GoalOccupancy>::iterator goalVertexLookupItem =
    goalVertexLookup.find(cell);
if (goalVertexLookupItem != goalVertexLookup.end() &&
    goalVertexLookupItem->second.i != i &&
    goalVertexLookupItem->second.arrivalTime <= t) {
    if (goalVertexLookupItem->second.arrivalTime < t) {
        occupants.insert(goalVertexLookupItem->second.i);
    }
    // At equality the owner's explicit path supplies its membership.
}
```

This retains the requested owner guard and `<=` condition while making the explicit/virtual distinction from `getCollisions.md`. The iterator name is consistently `goalVertexLookupItem`; `goalVertexItem` in the original example is a typo.

For `t > 0 && previousCell != cell`, compute a canonical `EdgeTime` and insert `i` into its forward or reverse set according to the actual move. A wait adds vertex occupancy only.

Do not emit conflicts during traversal, and do not use a boolean that suppresses vertex lookup after a goal hit. Sets deduplicate a virtual owner even when multiple visitors encounter it.

### Materialization

After all paths have been aggregated:

1. Keep vertex events with at least two participants.
2. For each participant, emit a `CellConflict` into `byAgent[i]` only when that timestep is explicit in its path. The virtual owner remains in the event.
3. Keep edge events only when both directional sets are nonempty; emit one `EdgeConflict` for each participant in their union.
4. Sort each per-agent list with the declared comparator.
5. Discard temporary nonconflicting occupancy buckets from the public result.

There is no need to pad short paths to the makespan. An explicit visitor triggers the goal lookup; two finished paths cannot collide because their destinations are unique.

Keep the existing public `getCollision` declaration in `include/mapf/utils.hpp` and implement the corrected detector directly in `src/mapf/utils.cpp`. Remove the `getCollisionV1` definition and any declarations or calls; do not add a V1 public API or a delegating `getCollision` wrapper. Solvers, validation, and reporting must all call the single `getCollision` implementation. If the optional benchmark is created, keep its independent slow pairwise oracle private to `tools/collision_benchmark.cpp`, converting its output to the new semantics; it is not a production detector or a retained V1 implementation.

## Repair scheduling

Add this enum to the shared solver header:

```cpp
enum class LocalRepairStrategy {
    RESOLVE_BY_AGENT,
    RESOLVE_BY_TIME
};
```

Append a defaulted strategy parameter to both constructors, preserving existing calls:

```cpp
LocalPathRepairParallelSolver(
    const Instance& instance,
    std::size_t numberOfThreads,
    bool continueIfFailed = false,
    LocalRepairStrategy localRepairStrategy =
        LocalRepairStrategy::RESOLVE_BY_AGENT
);

LocalPathRepairIterativeSolver(
    const Instance& instance,
    bool continueIfFailed = false,
    LocalRepairStrategy localRepairStrategy =
        LocalRepairStrategy::RESOLVE_BY_AGENT
);
```

Pass the strategy to `repairInitialPaths`; validate invalid enum values at the API boundary. Implement an actual `switch` in that function around selection, sharing the repair/commit body:

```cpp
switch (localRepairStrategy) {
case LocalRepairStrategy::RESOLVE_BY_AGENT:
    selected = selectByAgent(currentConflicts, ignoredForRevision);
    break;
case LocalRepairStrategy::RESOLVE_BY_TIME:
    selected = selectByTime(currentConflicts, ignoredForRevision);
    break;
default:
    throw std::invalid_argument("Unknown local repair strategy.");
}
```

- `RESOLVE_BY_AGENT`: choose the lowest path index with an eligible record, then its earliest record. Continue that agent until it has no eligible conflicts. After a commit, reselect from the current snapshot; if a changed suffix reintroduces a conflict involving a lower-index agent, revisit that index.
- `RESOLVE_BY_TIME`: inspect the first eligible record of each agent and choose the global minimum `(time, kind, coordinates, pathIndex)`. After every commit, select again globally. Sorting a single agent's list is not sufficient.
- Both modes select only explicit repair owners. Validate membership and event activity before an attempt. Rebuilding removes obsolete singleton vertices and edges without an opposite direction.
- Start with a linear comparison of per-agent heads, costing at most `O(A)` per selection after advancing ignored entries. A persistent heap is optional future work; snapshot replacement makes stale heap entries a separate concern.

For example, if agent 0 first conflicts at time 10 and agent 2 first conflicts at time 3, the agent strategy selects agent 0 and the time strategy selects agent 2.

`continueIfFailed=false` returns after local attempts and full fallback fail. With `true`, mark the selected `(pathIndex, event key)` ignored only for the unchanged committed revision and continue to eligible records/agents. Ignoring a record never removes its event from diagnostics or establishes success. Invalidate ignored selections on a commit.

Repeated-state protection must cover the entire committed path configuration and eligible selection, not only an agent-local fingerprint that is reset whenever scheduling revisits an agent. Reject repeated candidates and use fallback; if fallback fails, apply the continuation policy. This prevents the global scheduler from cycling between previously seen solutions.

## Bounded local repair and index consistency

### Explicit planning assumption

Use the rebuild option recommended in `getCollisions.md`: the **local bridge and suffix adjustment** inspect only the interval ending at the next conflict; a **global collision-index rebuild** still reads the provisional full paths once before a commit. This preserves detection correctness when an accepted segment changes later absolute times.

This is not a claim that all work per repair is bounded by the segment length. Full reservation/index rebuilds remain global costs and must be measured separately. The preference question presented during planning distinguishes this approach from forcing reconnection at the old absolute time; no user answer is recorded at creation.

A strict prohibition on reading the remaining suffix even for index maintenance would require preserving all its timestamps or a substantially more complex lazy/incremental representation. Merely decrementing the triggering event does not satisfy that stronger requirement.

### Window boundaries

From the current, sorted `byAgent[activeIndex]`:

1. Group all active records at the selected time `tc`; resolving just a vertex record while leaving an edge record at the same time is not progress.
2. Find the next active conflict time strictly greater than `tc`, called `tn`. Ignored unresolved records still count as physical boundaries.
3. Let `endOld` be `tn - 1`, or the last explicit index if there is no later conflict. The occurrence at `tn` belongs to the next repair.
4. Keep the initial prefix anchor at `tc - 1`. The usual first reconnection index is `tc + 1` for vertices and `tc` for edges. Apply the existing backward adaptive anchors to find a viable bridge.
5. Bound every potential-conflict scan and Scenario 2.3 transition loop by `endOld`. The current prefix anchor and the splice transition are included for correctness; do not start searching an unbounded suffix for potential conflicts.
6. If there is no usable reconnection before the next event, as can happen with consecutive conflicts, exhaust legal bounded anchors and invoke the existing full-path fallback. A conflict at the goal similarly needs final-segment handling or fallback.
7. When no next conflict exists, the bounded remainder naturally extends to the real goal and must establish permanent-goal safety.

Represent old path indices and new absolute arrival times separately. Waiting or detouring changes the latter; using `tn` unchanged as a new path index is incorrect.

Replace `buildLocalCandidate`/the suffix helper interface with an explicit bound, for example:

```cpp
struct RepairWindow {
    std::size_t prefixEnd;
    std::size_t reconnectIndex;
    std::size_t endOldInclusive;
    std::optional<int> nextConflictTime;
};

struct LocalCandidate {
    std::vector<Cell*> path;
    int validatedThrough;
    int suffixTimeShift;
};
```

A pending tail begins at old index `endOld + 1`; its new arrival time is its old index plus the accumulated shift. Copying/splicing that tail is allowed, but do not run SIPP or temporal suffix-validation loops over it as part of the local attempt. The rebuild discovers its new actual conflicts.

### Preserve the suffix scenarios within the bound

- Equal arrival: reuse the bounded part at its old timing.
- Early arrival with potential interference in the bounded part: wait safely to restore its timing.
- Early arrival without potential interference: permit a shift, then rebuild conflict indexes.
- Late arrival with potential interference: run the existing transition/SIPP procedure only through `endOld`.
- Late arrival without potential interference: reuse that bounded part with its delay.

Potential interference checks apply only to the bounded part; they say nothing about the unexamined tail. Every changed move still checks reverse edges, every wait stays in a safe interval, and any completed path requires a permanently safe goal. SIPP endpoint acceptance remains transient for an intermediate boundary.

The first unexamined tail position must occur strictly after `tc`; otherwise extend a safe wait within the bound or reject that candidate. Validate the changed prefix/bridge/segment and the structural connectivity of its splice. The first edge into the retained tail belongs to the global consistency check: a conflict there after `tc` may be the deliberately deferred next event and does not by itself invalidate this repair. Full structural validation of each original path happens once on entry; unchanged cell sequences inherit their structural validity.

### Transactional rebuild and progress

1. Keep the current `SolutionConflicts` and committed reservations for the current revision. Build the reservation table excluding the active agent once for the selected attempt; reuse it across anchors.
2. Construct and locally validate a bounded candidate without mutating committed structures.
3. Build one provisional collision snapshot for the candidate path set. This global step covers shifted tails, new conflicts, disappearing conflicts, virtual goal occupancy, and changed arrival times.
4. Use the provisional `byAgent` list to verify explicit progress, and check global event membership for any newly introduced virtual participation by the active agent. All selected-time conflicts must disappear, and no new active-agent conflict may appear at or before `tc`. For continuation, pre-existing earlier ignored conflicts may survive only unchanged; otherwise continuation would prevent repairs of later conflicts of the same agent. Such older conflicts must lie outside the changed, temporally validated segment; reject or repair them if an expanded anchor crosses them.
5. Reject a repeated state or a candidate that fails progress. Rejected objects are discarded. Do not decrement membership in the live snapshot.
6. Build the provisional committed reservation state once and atomically publish the path, conflict snapshot, reservations, costs, and revision. Reuse the provisional conflict snapshot; do not immediately call `getCollision` again.
7. Discard old list iterators/boundaries, clear revision-scoped ignored records, and select the next conflict under the chosen strategy.

A full-path SIPP fallback is a separate existing operation when bounded repair fails. It validates the complete replacement, uses permanent-goal policy, and must remove every conflict involving the active path, including virtual occupancy. It cannot silently be treated as another bounded local attempt.

Refactor `synchronizeResult` into snapshot construction and metric/finalization work so it does not rebuild collision/reservation data already valid for the current revision. A final authoritative validation is permitted once at return. Success requires all required paths and no active global event, including events involving a virtual owner with an empty repair list.

Incremental removal is deferred. The chosen rebuild satisfies the correctness recommendation without pretending that the present safe intervals retain reservation provenance.

## CLI and experiment output

Accept exactly:

```text
-localRepairStrategy RESOLVE_BY_AGENT
-localRepairStrategy RESOLVE_BY_TIME
```

Use the existing single-hyphen CLI convention and the requested camel-case flag name. Omission means `RESOLVE_BY_AGENT` for both local solvers. Reject unknown values, missing values, repeated occurrences, and use with `PriorityPlanningSolver`, following the existing parser's exit-code-2 behavior. Keep thread and continuation rules unchanged.

Example:

```bash
scripts/run_experiment.sh normal -- \
  -map benchmarks/maps/den520d.map \
  -scen benchmarks/scenarios/den520d/random/den520d-random-1.scen \
  -solver LocalPathRepairParallelSolver \
  -agents 100 -threads 8 \
  -localRepairStrategy RESOLVE_BY_TIME
```

Append `local_repair_strategy` to the stats CSV so experiments can be compared reproducibly. Store the effective enum name for local solvers and `-` for priority planning. Model the metadata as an optional strategy in `ExperimentRunResult`; default missing local metadata consistently to `RESOLVE_BY_AGENT` at normalization, and reject contradictory priority-planning metadata. Verify the resulting header against the documented schema.

Keep the existing grouped conflict CSV schema:

```text
cell_1,cell_2,timestep,conflict_type,agents
```

One row per global event remains a reporting projection of the new agent-indexed model. Map all global participants to real IDs, including virtual destination owners; sort IDs and rows deterministically. Do not produce a row per repair reference or expand pairs. Preserve the meaning of `paths_resolved` and per-agent success: a virtual owner is still involved in a collision even when its repair list is empty.

Rewrite `normalizeConflicts(instance, paths)` to call the shared detector and project its event maps. Remove the duplicate normalization from `runCli` and the example executables; the artifact writer computes final records once from its final paths. Remove the redundant `ExperimentRunResult::remainingConflicts` field if it remains unused by the writer after this refactor. `LocalPathRepairResult::initialConflicts` and `remainingConflicts` remain required snapshots using the new model.

The solution CSV, conditional conflict-file rules, and historical result files remain compatible. Document the additional stats column as a schema change.

## Files to create or change during implementation

All paths below are repository-relative. The current deliverable is only this plan. Test files and test-support files are excluded from the files to create or change; the validation criteria below do not authorize adding or editing them.

| Files | Work |
| --- | --- |
| `include/mapf/core/instance.hpp`, `src/mapf/core/instance.cpp` | Private `isValidInstance`, coordinate maps, validation in both constructors. |
| `include/mapf/core/cell_conflict.hpp`, `include/mapf/core/edge_conflict.hpp` | Assignable pointer/value per-agent records; canonical edge convention. |
| `include/mapf/core/solution_conflicts.hpp` | Membership maps, event types, mixed sorted agent lists, emptiness contract. |
| New `include/mapf/core/conflict_key.hpp`, `src/mapf/core/conflict_key.cpp` | Cell/time and edge/time key equality, hashes, and canonicalization shared by detection/selection. |
| `include/mapf/utils.hpp`, `src/mapf/utils.cpp` | Keep `getCollision` as the sole detector; implement validated aggregation, materialization, and sorting directly in it, and remove `getCollisionV1`. |
| `include/mapf/solvers/local_path_repair_solver.hpp` | Strategy enum and updated result contracts. |
| `include/mapf/solvers/local_path_repair_parallel_solver.hpp`, `include/mapf/solvers/local_path_repair_iterative_solver.hpp` | Strategy constructor/member. |
| `src/mapf/solvers/local_path_repair_parallel_solver.cpp`, `src/mapf/solvers/local_path_repair_iterative_solver.cpp` | Forward strategy; preserve only initial-search parallelism. |
| `src/mapf/solvers/local_path_repair_solver_common.hpp`, `src/mapf/solvers/local_path_repair_solver_common.cpp` | Switch selection, bounded windows, revisions, transactional rebuilds, progress/continuation handling; remove duplicate endpoint checks and global ownership scans. |
| `src/main.cpp` | CLI parsing, dispatch, help, strategy metadata, remove duplicate normalization. |
| `src/mapf/experiments/experiment_utils.hpp`, `src/mapf/experiments/experiment_utils.cpp` | Shared-detector projection, virtual-owner status, strategy metadata/CSV. |
| `src/mapf/experiments/manual_experiment.cpp`, `src/mapf/experiments/benchmark_experiment.cpp` | Adapt experiment result initialization and normalization changes. |
| `src/mapf/solvers/priority_planning_solver.cpp` | Audit boolean validator assumptions; adapt only if its call contract requires it. |
| New `tools/collision_benchmark.cpp` | Optional dependency-free comparison of the old pairwise oracle and new detector. |
| `CMakeLists.txt` | Register the key implementation and an opt-in collision benchmark target; no new test targets. |
| New `docs/collision_index_and_repair_strategies.md` | Model, scheduling, bounded local work versus global rebuilds, API migration, benchmark procedure. |
| `docs/local_path_repair_parallel_solver.md`, `docs/local_path_repair.md`, `docs/path_sufix_repair_algorithm.md` | Update implementation guidance/bounds; clearly mark historical whole-suffix examples as superseded. |
| `docs/experiment_cli_and_results.md`, `readme.md` | CLI/default examples, input restrictions, stats column and export semantics. |
| `.github/mapf.md` | Clarify that starts and goals are independently unique and invalid instances throw before search; retain the user's new constraints. |
| Future `specs/007-collision-index-and-local-repair-strategies/spec.md` | Derive the executable implementation contract from this plan when specification work is requested. |

Existing `CMakePresets.json`, run/profiling scripts, SIPP search APIs, and `Result` metrics require no behavioral changes for the selected design. Profile the existing entry points and forward the new CLI flag through the scripts unchanged.

## Validation plan

### Instance and detector correctness

Use these scenarios as correctness criteria for implementation review, existing compatible checks, and optional benchmark validation. Creating or editing test files is outside the planned file changes. Required cases:

- Duplicate start only, duplicate goal only, and both duplicates throw `std::invalid_argument` in manual and file constructors.
- Unique endpoints, zero agents, one agent at its own goal, crossed start/goal pairs, repeated scenario buckets, and sparse real agent IDs remain supported.
- Duplicate endpoints passed directly to `getCollision` are rejected; the boolean wrapper returns false. An empty slot between live paths does not alter ownership.
- A single path, including waits and one-cell paths, has no self-conflict.
- Goal owner processed first or last yields equivalent events after remapping path indexes.
- Arrival equality versus later virtual occupation yields the declared different repair-record counts.
- Two visitors meeting a parked owner produce one event and two repair records; no discovery-order duplicates.
- Two, three, and four explicit agents meeting at an intermediate cell produce exactly two, three, and four repair records. Check identities as well as counts.
- Opposite traversals produce one global event and two records; three-versus-one direction multiplicity produces four records. After replacing the only reverse traversal, recomputation drops the edge event while retaining any independent vertex conflicts.
- Waits never create swaps; same-direction travel, following, and allowed cycles respect the defined distinction.
- Vertex and edge records at the same time are both preserved and ordered; per-agent lists discovered out of time order are sorted.
- Unequal path lengths, long waits, null pointers, and safe integer-conversion boundaries have explicit outcomes.

Some existing collision, export, and solver checks assume shared endpoints or the old pairwise representation. Record incompatibilities with the new contract when evaluating the existing suite; updating those test files is outside this plan's file-change scope. Inputs used to validate intermediate meetings must have distinct final destinations, and duplicate-endpoint scenarios must be evaluated as constructor rejection cases.

For differential validation in the optional benchmark, generate deterministic grid walks with unique full-path endpoints, waits, revisits, empty slots, and different lengths; compare canonical events, directional membership, and explicit owner lists to its independent `O(A²H)` oracle. Normalize pairwise observations into participant sets so the oracle checks the new contract instead of comparing legacy counts. Keep this support inside the benchmark file, without adding a test-support file.

### Local repair and scheduling

- Prove the two selection orders using disjoint conflict groups with different times and path indexes; verify deterministic tie handling.
- The parallel and iterative adapters produce identical paths/metrics for the same strategy, excluding elapsed time; multiple thread counts do not change ownership.
- For one agent with multiple separated conflicts, a local attempt only evaluates through the next distinct conflict boundary.
- Exercise all bounded suffix scenarios, early/late shifts, a newly introduced collision in the shifted tail, and disappearance of a previously scheduled event.
- Check consecutive conflicts, a conflict at the goal, same-time vertex/swap events, and collision with a virtual destination owner.
- Validate changed arrival times, permanent goals, edge reservations, and ownership after every accepted commit.
- Snapshot replacement invalidates stale selections; copies of initial diagnostics remain unchanged.
- Failed local candidates and failed full fallback leave the committed solution/index/reservations intact.
- Continuation handles independent components and later conflicts of the same agent after an ignored earlier failure; it never hides unresolved events.
- Global scheduling revisits lower path indexes when necessary and terminates on repeated states.
- Final metrics, missing paths, zero-cost paths, and failure diagnostics remain correct.

During performance validation, distinguish `local suffix transitions examined`, `detector explicit cells read`, and `reservation rebuilds` using profiling or private measurement counters. A bounded-work check must distinguish these costs; do not claim no suffix scans simply because the final path is valid. Expose no new public runtime logging solely for validation.

### CLI and reporting

- Omitted strategy equals explicit `RESOLVE_BY_AGENT`; both enum strings work for both local solvers.
- Unknown, missing, duplicate, and priority-planning strategy flags fail with code 2.
- Invalid scenario endpoints fail before search and create no result bundle, with either continuation setting.
- Stats contain the effective strategy and match the documented schema exactly.
- Grouped conflict rows retain all sorted real IDs, including parked owners; sparse IDs cannot be mistaken for path indexes.
- Success remains code 0, algorithmic failure code 1, and failed local runs with no geometric conflicts still get header-only conflict files.
- Final output remains deterministic and generated historical files are not rewritten.

### Commands for the implementation phase

```bash
cmake --preset normal
cmake --build --preset normal --parallel
ctest --preset normal
cmake --preset profile
cmake --build --preset profile --parallel
```

Attempt the existing normal suite after implementation, including SIPP, priority planning, CLI, and script checks. Record any build or assertion incompatibilities caused by the excluded test migration; do not report those checks as passing or modify test files to resolve them within this scope. Validate the application and optional benchmark separately if legacy test compilation blocks the aggregate build. Use the profiling workflow from specification 006 for representative experiments. These are planned checks, not tests already passed by this document.

## Performance evaluation and limits

Let `A` be the number of path slots, `S` the total number of explicit path cells, `H` the makespan, and `C_i` the number of repair records for path `i`.

- Old detection costs `O(A²H)`.
- The new aggregation/materialization has expected cost `O(A + S + C)`, with `C = sum(C_i)`, followed by `O(sum(C_i log C_i))` per-agent sorting.
- Peak detector memory is `O(A + S + C)`; it does not allocate an `A * H` padded occupancy table.
- Hash-table complexity is expected, not a worst-case guarantee.
- Selection compares agent heads; global index/reservation rebuilding and path copying remain additional costs.
- A four-agent event saves pairwise expansion, but participant sets and per-agent records can increase memory on two-agent-heavy workloads. Measure rather than assume a memory improvement.

For the optional benchmark, keep the slow detector outside the production library, generate the same valid inputs once, and exclude generation/I/O from timing. Include collision-free paths, dense multi-agent meetings, asymmetric path lengths with parked goals, and both edge directions. Compare canonical correctness before timing; consume the result to prevent dead-code elimination. Use warmup plus repeated runs, record medians/spread, compiler flags, hardware, input seed, `A/S/H`, event/record counts, and memory.

Profile both strategies on identical instance prefixes and thread counts. Compare the updated `getCollision` against the benchmark's pairwise oracle on identical paths; strategy changes may change paths and success rates, so they are not a clean detector speed comparison. Report local-attempt work, index rebuild time, reservation time, and total solve time separately.

Acceptance requires correct equivalent events and a demonstrated detector improvement on representative larger workloads; no numeric speedup is promised before measurements. If end-to-end time is dominated by reservation rebuilding, document that result.

## Implementation sequence and completion criteria

1. Write the follow-up specification with the contracts and replacements above; retain this plan's audit/prompt history.
2. Implement endpoint validation and establish the new input domain before enabling the single-owner goal map.
3. Introduce keys/event membership/per-agent structs using explicit types without aliases; update `getCollision` directly and remove `getCollisionV1`. Update all production callers so the model migration builds coherently.
4. Establish detector correctness against the validation criteria, using the independent oracle if the optional benchmark is created.
5. Migrate experiment normalization and shared result consumers.
6. Implement the strategy enum, constructor forwarding, switch-based scheduling, and continuation/revision rules.
7. Bound local candidate processing, reuse one provisional conflict snapshot per commit, and remove repeated pairwise progress scans.
8. Add CLI/statistics metadata, update examples/docs, and run the existing compatible correctness checks, recording legacy test incompatibilities without changing test files.
9. Measure the detector and both strategies using normal/profile builds; record tradeoffs and remaining hotspots.

The feature is complete when both constructors reject duplicate endpoints, `getCollision` is the only production detector and `getCollisionV1` has been removed, the implementation uses explicit types without introducing aliases, all production consumers use the new detector contract, per-agent records/membership are correct, both strategies are selectable with the required default, local suffix adjustment stops at the next conflict, global consistency survives changed timestamps, exports retain correct participant identities, and validation/performance evidence is recorded. Test-file creation and modification are excluded from these completion criteria; any resulting gaps in automated validation must be documented.

## TODO

TODO: After profiling, evaluate compact participant storage for mostly two-agent events without weakening membership or directional semantics.

TODO: Consider a global time-priority heap only if head selection is measurable; include snapshot revisions in its design.

TODO: If rebuilds dominate runtime, design a separate incremental occupancy/reservation system with per-agent provenance, old/new goal-arrival updates, affected-neighbor discovery, and differential tests. Decrementing one scalar event is insufficient.

TODO: Consider persistent/indexed path storage if copying the unmodified tail remains costly. Bounding local temporal evaluation alone does not eliminate list/vector copying.

TODO: Investigate fixed-time reconnection as an alternative when avoiding global tail scans is more important than preserving flexible local delays; it changes which local repairs can succeed.

## adjustments

Later numbered adjustments override earlier conflicting requirements and the preserved original prompt.

### adjustment 1

Adjusted at: 2026-09-16T19:40:34-03:00

#### User request

```text
faça as os seguintes ajuste no plano:

* não use alias para tipos de variáveis (ex: usando "using"), deixe os tipos padrão, mesmo que fiquem enormes
* a função getCollisionV1 e os ajustes propostos servem apenas de base para como deve ser a função getCollision, que deve existir uma somente (fazer os ajustes na função getCollision e retirar getCollisionV1)
* tire os testes dos arquivos que tem que ser editados ou adicionados

guarde esses ajuste também na seção "adjustments" do plano
```

#### Changes

- **Before:** the representation examples introduced `PathIndex`, `Participants`, and `AgentConflict` aliases and used inferred local types in the occupancy pass. **After:** examples use `int`, `std::unordered_set<int>`, `std::variant<CellConflict, EdgeConflict>`, complete nested containers, and an explicit iterator type directly. The implementation must not introduce type aliases with `using`, `typedef`, or equivalent shortcuts.
- **Before:** the plan exposed `getCollisionV1` publicly and made `getCollision` delegate to it. **After:** the V1 draft and its proposed corrections serve only as a basis for updating `getCollision` directly. Remove `getCollisionV1` and retain a single production collision detector, with API, algorithm, file inventory, performance comparison, and completion criteria updated consistently.
- **Before:** the file inventory included existing and new test files, a shared test-support header, and CMake registration of new tests; validation and implementation steps instructed fixture/test updates. **After:** remove all test and test-support files from the create/change inventory and remove new test registration. Preserve correctness scenarios and execution of existing compatible checks as validation criteria, without scheduling test-file edits; keep any optional oracle inside the benchmark and document legacy test incompatibilities.
- **Before:** `Last updated` was `2026-09-15T21:31:47-03:00`, and the adjustments section had no recorded corrections. **After:** `Last updated` is `2026-09-16T19:40:34-03:00`, and this entry records the complete request and before/after changes while preserving the original prompt.
