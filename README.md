# MDFSMVRP_model_comparison

## Introduction:
This repository implements the three MDFSMVRP models introduced and discussed in the paper of Lahyani et al.(2018) using Gurobi Python API and compared the results of the three models. 

### Problem description:
The problem discussed here is the multi-depot fleet size and mix vehicle routing problem (MDFSMVRP). The solution of the problem can be defined as a directed graph $G = (V, A)$ with V being the vertex set and A being the arc set. 

### Mathematical formulation:
- Each customer $i \in V_c$ has demand $q_i \geq 0$; each depot $i \in V_d$ has zero demand $q_i$ = 0
- Distance between $i,j \in V$ is $\beta_{ij}$ ; here we assume cost is positively reltated to distance
- Loop arcs $(i,i)$ is not allowed: $\beta_{ii} = \inf \quad \forall i \in V_c$
- Depots cannot be connected to depots: $\beta_{ij} = \inf \quad \forall i,j \in V_d, i \neq j$
- Different depots house heterogeneous vehicles $K = {1,...,K}$ with accroding capacity $Q^K$, fixed cost $F^K$, and a variable cost $\alpha^K$
