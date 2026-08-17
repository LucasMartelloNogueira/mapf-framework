# Multi-Agent Path Finding (MAPF)

## Purpose

This document describes the classical **Multi-Agent Path Finding (MAPF)** problem independently of any specific algorithm.

Its purpose is to provide context for implementing and modifying MAPF algorithms while standardizing the terminology used throughout the project.

The definitions presented here are primarily based on the following paper:

> **Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks**  
> Roni Stern et al.  
> Symposium on Combinatorial Search (SoCS), 2019.

---

# Problem Description

The **Multi-Agent Path Finding (MAPF)** problem consists of finding collision-free paths for multiple agents that must move simultaneously within a shared environment.

Each agent has:

- an initial position;
- a target location;
- the ability to move through the environment.

The objective is to produce a set of paths such that:

- every agent reaches its assigned destination;
- no conflicts occur during the simultaneous execution of the paths.

In its classical formulation, MAPF assumes that:

- the environment is represented as an undirected graph;
- time is discrete;
- each action takes exactly one time step;
- at each time step, an agent may:
    - remain in its current position (wait);
    - move to an adjacent vertex (move).

---

# Problem Input

A classical MAPF instance is defined by the tuple:

```
(G, s, t)
```

where:

## Graph (G)

```
G = (V, E)
```

- **V** is the set of vertices.
- **E** is the set of edges.

The graph models the navigable environment.

Depending on the application, the graph may represent:

- 4-neighborhood grids;
- 8-neighborhood grids;
- warehouse layouts;
- urban road networks;
- arbitrary graphs.

---

## Number of Agents

Let

```
k
```

denote the number of agents.

Each agent is identified by an index:

```
1 ... k
```

---

## Initial Position

The function

```
s(i)
```

defines the initial vertex of agent *i*.

Formally:

```
s : {1,...,k} → V
```

---

## Target Position

The function

```
t(i)
```

defines the target vertex of agent *i*.

Formally:

```
t : {1,...,k} → V
```

---

# Problem Output

The expected output consists of two components.

## 1. Existence of a Solution

The algorithm must determine whether a conflict-free set of paths exists.

Therefore, the result is either:

- a valid solution;
- or a determination that the instance is unsolvable.

---

## 2. Set of Plans

If a solution exists, it consists of one plan for each agent.

A plan is an ordered sequence of actions:

```
πᵢ = [a₁, a₂, ..., aₙ]
```

Each action can be:

- Move
- Wait

Executing the plan moves the agent from its initial position to its target.

The complete solution is the set:

```
Π = {π₁, π₂, ..., πₖ}
```

All plans are executed simultaneously.

---

# Conflicts

A solution is considered valid only if no conflicts occur between any pair of agents.

The exact definition of a conflict depends on the specific MAPF variant being considered.

The most common conflict types are described below.

---

## Vertex Conflict

A vertex conflict occurs when two agents occupy the same vertex at the same time.

Example:

```
Time 5

Agent A -> V3
Agent B -> V3
```

This is the most commonly considered conflict in MAPF algorithms.

---

## Edge Conflict

An edge conflict occurs when two agents traverse the same edge at the same time and in the same direction.

Example:

```
Time 7

A: V1 -> V2

B: V1 -> V2
```

---

## Swapping Conflict

Also known as an **edge swap**.

Two agents simultaneously exchange positions.

Example:

```
Time t

A: V1 -> V2

B: V2 -> V1
```

Although the agents never occupy the same vertex simultaneously, this movement is generally considered invalid.

---

## Following Conflict

A following conflict occurs when one agent enters a vertex immediately after another agent leaves it.

Example:

```
Time t

A occupies V5

Time t+1

B enters V5
```

Not all MAPF variants consider this behavior to be a conflict.

---

## Cycle Conflict

A cycle conflict involves three or more agents.

Each agent moves into the position previously occupied by another agent, forming a cycle.

Example:

```
A -> position of B

B -> position of C

C -> position of A
```

Some MAPF variants allow this behavior, while others prohibit it.

---

# Relationship Between Conflict Types

Some conflict types naturally imply others.

For example:

- Prohibiting vertex conflicts also prevents edge conflicts.
- Prohibiting following conflicts also prevents cycle conflicts.
- Prohibiting cycle conflicts also prevents swapping conflicts.

Therefore, every MAPF algorithm must explicitly define which conflict types are considered invalid.

---

# Objective Functions

Multiple valid solutions may exist for the same MAPF instance.

Therefore, an objective function is required to determine which solution is preferred.

The two most common objective functions in the literature are:

---

## Makespan

Makespan represents the time step at which the last agent reaches its destination.

Formally:

```
Makespan = max(|π₁|, |π₂|, ..., |πₖ|)
```

Objective:

```
Minimize the total completion time of the mission.
```

This objective is appropriate when all agents should finish as early as possible.

---

## Sum-of-Costs (Flowtime)

The Sum-of-Costs represents the sum of the lengths of all agent plans.

Formally:

```
SOC = |π₁| + |π₂| + ... + |πₖ|
```

Objective:

```
Minimize the total accumulated cost across all agents.
```

This objective favors globally efficient solutions, even if an individual agent finishes later than others.

In the literature, it is also referred to as:

- Flowtime
- Total Cost

---

# Agent Behavior After Reaching the Goal

After an agent reaches its destination, two common conventions are used.

## Stay at Target

The agent remains at its target vertex until every other agent has also reached its destination.

As a result, no other agent may pass through that vertex.

This is the convention most commonly adopted in classical MAPF algorithms.

---

## Disappear at Target

As soon as the agent reaches its destination, it is removed from the environment.

Consequently, it no longer participates in conflicts.

This convention simplifies some problem variants and reduces the search space.

---

# Summary

A classical MAPF instance consists of:

- a graph;
- a set of agents;
- an initial position for each agent;
- a target position for each agent.

The objective is to find a set of simultaneous paths that:

- guide every agent to its destination;
- satisfy the chosen conflict constraints;
- optimize a specified objective function (typically Makespan or Sum-of-Costs).

Although many MAPF variants exist, the concepts described in this document form the common foundation shared by most algorithms in the literature.