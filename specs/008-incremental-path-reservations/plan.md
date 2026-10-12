Created: 2026-09-29T13:30:17-03:00 (original plan filesystem creation time)

Author: Lucas Martello Nogueira (local user: lucas; repository Git identity)

Last updated: 2026-09-29T14:42:09-03:00

AI model: GPT-6 (Codex)

# Plan: Incremental Path Reservations

## Objective and context

Implement reservation updates proportional to affected cells and movements while retaining one full state copy per repair attempt, as requested. Copy optimization is deferred. The [consolidated documentation](../../docs/incremental_path_reservatons.md) explains the problem, design decisions, examples, performance limits, and deferred alternatives. This plan organizes the work; the approved [specification](spec.md) defines its implementation contracts.

Sources: [project instructions](../../.github/ai-instructions.md), [MAPF constraints](../../.github/mapf.md), [consolidated problem analysis and improvement approaches](../../docs/incremental_path_reservatons.md), and [feature 007](../007-collision-index-and-local-repair-strategies/spec.md). The consolidated document incorporates both reviews, the user's responses, and the original plan; later decisions supersede earlier suggestions for an overlay or a separate occupancy index. Historical prompts below remain verbatim.

Preserve discrete time, closed intervals, unique starts and goals, permanent goal occupancy, both repair strategies, and existing candidate validation. Following moves and cycles remain allowed. Changes to reservations apply to both solver adapters through their shared sequential repair engine.

## Existing architecture and reusable components

`buildReservationState` constructs reservation tables; it does not execute SIPP. Before this feature, the common engine built the initial state, rebuilt while excluding an active path, and rebuilt after acceptance. Full fallback additionally copied other paths and called a SIPP overload that constructed another table.

Reuse `PathReservationState`, `SafeIntervalTable`, `EdgeKey`, `mergeBlockedIntervals`, and existing interval complement logic. Preserve `evaluateCandidate`, revision/cycle checks, continuation behavior, and publication of consistent paths/conflicts/reservations. `SolutionConflicts` contains only conflict events and cannot supply all occupancy data.

## Files to change or create during implementation

| File | Planned change |
| --- | --- |
| `include/mapf/solvers/local_path_repair_solver.hpp` | Temporal `vertex_agents` and explicit goal-owner type. |
| `src/mapf/solvers/local_path_repair_solver_common.cpp` | Initial builder migration, local interval regeneration, remove/add helpers, spatial membership lookup, loop integration, and removal of unused `pathsExcept`. |
| `src/mapf/solvers/local_path_repair_solver_common.hpp` | Internal reservation helper declarations needed by focused tests; keep them in `local_path_repair_detail`. |
| `include/mapf/pathfinding/a_star_sipp.hpp`, `src/mapf/pathfinding/a_star_sipp.cpp` | Public goal policy and table overload accepting that policy, with existing overload behavior preserved. |
| `tests/solvers/path_reservation_state_test.cpp` (new), `CMakeLists.txt` | Register focused full-rebuild equivalence and affected-entry tests. |
| `tests/pathfinding/a_star_sipp_test.cpp` | Permanent-goal search using a supplied table. |
| `tests/solvers/local_path_repair_parallel_solver_test.cpp`, `tests/solvers/local_path_repair_iterative_solver_test.cpp` | Update map assertions and exercise commit/failure behavior and both strategies. |
| `docs/local_path_repair_parallel_solver.md` | Replace full-rebuild descriptions with the new lifecycle, ownership rules, API, and cost limitations. |
| `docs/incremental_path_reservatons.md` | Consolidated reference for the problem, accepted approach, correctness examples, performance assessment, and deferred improvements. |

The data layout of `SafeIntervalTable` remains suitable. Keep the detector's private `GoalOccupancy` in `utils.cpp`; reservation ownership will use a separate, explicit type.

## Libraries and documentation researched

Keep C++20, the standard library, CMake/CTest, and existing thread support. No new runtime dependency is required.

| Facility | Choice and reference |
| --- | --- |
| `std::unordered_map` / `std::unordered_set` | Match the requested temporal indexes and owner sets. Iteration is unordered; never use it as time order. [C++ container requirements](https://eel.is/c++draft/unord.req). |
| `std::vector`, `std::sort`, `std::list` | Collect and sort affected-cell intervals; traverse full paths with a timestep and previous-cell pointer. [C++ sorting specification](https://eel.is/c++draft/alg.sort). |
| CMake/CTest | Extend the existing test targets and run regression checks. [CTest manual](https://cmake.org/cmake/help/latest/manual/ctest.1.html). |
| Boost ICL (considered, not selected) | Provides interval containers, but adding a dependency would not remove the need for owner tracking and edge/goal handling. The existing interval representation suffices for this first version. [Official documentation](https://www.boost.org/doc/libs/latest/libs/icl/doc/html/index.html). |

## Implementation sequence

### 1. Record temporal ownership

Use the requested nested map and a goal record storing the same real agent identity:

```cpp
// In namespace mapf, in local_path_repair_solver.hpp.
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

The adjustment from reusing `GoalOccupancy` is necessary because that type is private to `utils.cpp` and its `i` means path index. For example, agent ID `42` may occupy path slot `0`. Obtain reservation identity from `agents[activeIndex].id` and keep conflict ownership indexed by path as before.

Adapt `hasOtherAgent` to inspect each timestep's set and return immediately upon finding another ID. Preserve explicit final-cell membership; permanent occupancy is stored separately, without expanding infinite timesteps. Update all consumers: `vertex_agents[cell].contains(0)` now tests time zero, not agent zero.

### 2. Keep the initial builder and extract cell regeneration

Retain `result.reservations = buildReservationState(grid, agents, result.paths)` before the loop (currently line 985). Populate the new maps and preserve this full builder as the equivalence reference.

Extract a cell-local helper: collect `[t,t]` for nonempty owner sets, add any `[arrivalTime, SAFE_INTERVAL_INFINITY]` goal reservation, merge blocked intervals, and compute their complement. Run it once per distinct affected cell after each mutation batch. Preserve `[0, SAFE_INTERVAL_INFINITY]` for a fully free cell and an empty safe vector for a fully blocked one. Keep checked time conversions and infinity handling.

Example: removing the owner parked at `A` since time 4, while another agent visits at time 7, produces `[0,6]` and `[8,+inf]`. Extending the old interval directly to infinity would erase the remaining visit. Conversely, removing a visitor must preserve another agent's parked occupancy.

### 3. Remove and add full-path contributions

Implement these internal mutating helpers in the common engine:

```cpp
void repairSafeIntervalTable(
    PathReservationState& state, const std::list<Cell*>& oldPath, int agentId);
void updateReservationState(
    PathReservationState& state, const std::list<Cell*>& newPath, int agentId);
```

Both require complete paths starting at absolute time zero and matching ownership. Empty paths contribute nothing. Removal uses the currently registered path; insertion requires that this agent's old contribution has already been removed. Do not silently remove a different owner's goal. Update owners even when a safe interval does not change; erase empty occupancy entries, but retain fully free safe-table entries.

`repairSafeIntervalTable` removes explicit visits and the agent's permanent reservation. `updateReservationState` adds the new visits and goal arrival. Each then rebuilds only its affected cells. Full-path replacement covers changed geometry and shifted suffix times: two inserted waits shift all later reservations by two timesteps.

Update reverse-edge reservations for the affected movements as well. A move `U -> V` arriving at `t` blocks `{V,U}` at `t`; waits create no edge entry. After removing all explicit visits of the excluded path, preserve that edge-time block if the owner sets at `(U,t-1)` and `(V,t)` still intersect. This uses the requested vertex index without another owner map. For example, if X and Y share the movement, removing X preserves Y's block. Use non-inserting lookups for these queries and remove empty edge-time sets.

### 4. Reuse the table with the correct SIPP goal policy

Move `AStarSippSolver::GoalOccupation` to its public API and add the six-argument overload shown below. Keep the existing table overload transient and the path-vector overload permanent. Delegate all searches to the existing `solveWithTable` implementation.

```cpp
const auto fullPath = sipp.solve(
    grid, oldPath.front(), oldPath.back(), repairState.safeIntervalTable,
    0, AStarSippSolver::GoalOccupation::Permanent);
```

This overload is required: the current public table overload always uses `Transient`. If the goal is safe during `[0,4]` and `[6,+inf]`, a transient search may arrive at 3, but parking there conflicts at 5. Permanent search must find arrival at 6 or later, provided a safe route exists. Rejecting the earlier result afterward does not make SIPP search for that later solution.

### 5. Integrate one copied state per attempt

Replace the exclusion rebuild (currently line 1028) with a copy and removal. Reuse that table for all anchors, suffix repairs, and full fallback (currently line 1049). Only an accepted complete candidate may be added. The lifecycle is:

```cpp
// Integration sketch: keep this temporary alive through acceptance.
auto repairState = result.reservations;
const int agentId = agents[activeIndex].id;
repairSafeIntervalTable(repairState, result.paths[activeIndex], agentId);

// Existing searches and evaluateCandidate run here, reading the stable table.
// On rejection: discard repairState and retain current continuation behavior.
// On acceptance:
updateReservationState(repairState, accepted->paths[activeIndex], agentId);

// Complete revision checks and any allocating history work before publication.
result.paths = std::move(accepted->paths);
result.remainingConflicts = std::move(accepted->conflicts);
result.reservations = std::move(repairState);
// Retain existing fingerprint, revision, cursor and ignored-conflict updates.
```

Replace the post-acceptance full rebuild (currently line 1070) with the addition above. Adjust the temporary's current `const` declaration and block scope so it survives until commit; create it only for an actual attempt. Prepare all potentially throwing work before publishing the new state and preserve the existing transaction guarantees.

Adding directly to the original state would retain obsolete reservations: an old `A@5` and a new `B@5` would both remain blocked. Keep the copied table unchanged during searches because SIPP nodes retain interval indexes. Use the complete accepted path, including any shifted tail, and keep `evaluateCandidate`'s global checks.

### 6. Validate behavior and measure the intended work reduction

Compare exclusion against a full rebuild excluding the same path, and replacement against a full rebuild of updated paths. Check all four reservation views, normalized intervals, identities, edge directions, and arrival times. Add explicit expected-result cases so shared rebuilding logic cannot hide a common error.

Cover shared cell/edge occupancy, permanent owners and visitors, interval splitting/merging, waits/revisits, shifted tails, noncontiguous IDs, empty/unit paths, supported time bounds, and untouched cells. Rejected candidates must leave the committed result unchanged. Exercise permanent fallback, both strategies, and parity between iterative/parallel adapters. Update existing assertions to read `arrivalTime` and temporal owner sets.

Build production and test targets and run CTest. Measure identical optimized workloads before/after: initial building, copies, exclusion, insertion, SIPP, conflict detection, total time, and memory. Acceptance requires one initial full reservation build, no full reservation rebuild inside the repair loop or its fallback, and regeneration only of cells belonging to the removed/added paths. Copying the state and scanning other occupants of affected cells still cost time; no numerical speedup is assumed.

## Deferred improvements

- TODO: Replace full state copying with an overlay after correctness and profiling justify it.
- TODO: Consider a spatial owner cache if scanning temporal sets makes `hasOtherAgent` expensive.
- TODO: Consider shared interval-building code with `AStarSippSolver::getSafeIntervalsByCell` after validating this migration.
- TODO: Profile full-path copies, fingerprints, and conflict detection separately before extending incremental updates to them.

## prompt

### Original plan request

```text
leia o arquivo task.md e faça o que se pede
```

### Adaptation request

```text
leia o arquivo .github/ai-instructions.md e adapte o arquivo de plano criado para uma pasta em specs, botando em arquivo chamado plan.md
```

### Original task definition

```text
A seguir estão algumas anotações sobre uma melhoria no código, para evitar trabalho desnecessário e melhorar o desempenho

==================================

# MELHORA 3

problema: loop de buildReservationState faz SIPP novamente para vários caminhos de forma desnecessários
* arquivo: src/mapf/solvers/local_path_repair_solver_common.cpp
* não precisa refazer todo o buildReservationState para resolução de conflito (dentro do loop while(true)).
* objetivo: não refazer todos os safe intervals de células que não são afetadas pelos excludedPath, pois isso torna a função cara
* criar uma função chamada RepairReservationState, no qual os parametros são um grid, um PathReservationState, e um excludedPath (um caminho do tipo std::list<Cell*>). o Objetivo dessa função é pegar um PathReservationState e refazer os safe Intervals das células que estão no caminho de excludedPath. Por exemplo:

supondo que o safe interval da célula A seja: [(0, 4), (7, 9), (12, +inf)]
os intervalos de ocupação da célula A são: [(5, 6), (10, 12)]
agora suponha que excludedPath tenha a célula A no instante 5 (célula A ocupa posição de índice 5 - elemento na posição 6 - na lista do caminho)
Sabendo disso, e contanto que nenhum outro agente passa pela célula A no instante 5, podemos adaptar o safe interval da célula A para [(0, 5), (7, 9), (12, +inf)]
e repetimos o processo para todas as células do caminho em excludedPath.

Nesse mesmo exemplo, caso outro agente passe pela célula A no instante 5, o safe interval não muda

teriamos só que adaptar o algoritmo para armazenar, em cada célula, os instantes e os agentes que ocupam aquela célula. Sugira locais onde fazer isso (em qual classe e qual função)

==================================

A seguir estão alguns arquivos que foram trocados entre mim e o codex, leia eles para entender o contexto da funcionalidade pedida, dos seus objetivos e dos problemas que ainda existem
* seção acima (MELHORA 3): feita por mim
* melhoria_3.md: feita pelo codex, depois de pedir para analisar este arquivo (somente a seção MELHORA 3)
* melhoria_3_resposta.md: minha resposta depois de ler melhoria_3.md
* melhoriaa_3_1.md: reposta do codex depois de pedir para avaliar novamente melhoria_3_reposta

com esse contexto em mãos, monte um plano para implementar a funcionalidade pedida. Deixe o plano o mais resumido possível sem perder clareza. Tente seguir ao máximo as abordagens sugeridas por mim, se isso não for possível, de uma exemplo de um problema de porque ela não pode ser seguida e algumas sugestões de como prosseguir
```

## adjusments

### adjustment 1

Datetime: 2026-09-29T13:38:08-03:00

Full user prompt:

```text
leia o arquivo .github/ai-instructions.md e adapte o arquivo de plano criado para uma pasta em specs, botando em arquivo chamado plan.md
```

| Item | Before | After |
| --- | --- | --- |
| Plan location | `plano_melhoria_3.md` at repository root. | Adapted plan at `specs/008-incremental-path-reservations/plan.md`; original retained as provenance. |
| Language | Portuguese. | English, with verbatim prompts preserved in their original language. |
| Audit | No metadata or adjustment history. | Original creation time, sourced author identity, update time, AI model, and this adjustment record. |
| Implementation guidance | Six concise prose steps. | Same approach with file inventory, C++ examples, researched dependencies, validation criteria, and deferred TODOs. |
| Scope and decisions | Copy per attempt; temporal owners; remove/add functions; permanent fallback. | Preserved; no feature implementation or specification added by this adaptation. |

### adjustment 2

Datetime: 2026-09-29T14:42:09-03:00

Full user prompt:

```text
junte os seguintes documentos: melhoria_3.md, melhoria_3_resposta.md, melhoria_3_1.md e plano_melhoria_3.md em um unico arquivo em docs/incremental_path_reservatons.md, no qual detalha o problema e as abordagens para melhorar o desempenho do programa. Depois disso, atualize os arquivos em specs/008-incremental-path-reservations para apontar para esse arquivo de documentação
```

| Item | Before | After |
| --- | --- | --- |
| Problem and design references | Separate links to the original plan, initial review, user responses, and follow-up review. | One consolidated reference at `docs/incremental_path_reservatons.md`, using the requested filename. |
| Documentation inventory | General local-repair solver documentation only. | Also includes the consolidated problem, approaches, examples, performance assessment, and deferred work. |
| Context wording | Described the original rebuilding behavior as current and the specification as future work. | Identifies that behavior as the pre-feature baseline and links the approved specification. |
| Approved contracts and history | Copy per attempt, temporal owners, remove/add helpers, permanent fallback, and verbatim prompts. | Preserved; the consolidation changes documentation references without changing implementation requirements. |
