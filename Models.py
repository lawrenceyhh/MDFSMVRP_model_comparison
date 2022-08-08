import gurobipy as gp
from gurobipy import GRB
import numpy as np
import math


def run_F3(demands, vertices, arcs, V_d, V_c, F, alpha, K, Q, runtime_limit=1800, print_out=False):
    model = gp.Model("Compact formulation with loading variables")

    # supress console output from Gurobi
    if not print_out:
        model.Params.LogToConsole = 0

    # Model Parameters(Stopping Condition)
    model.Params.TimeLimit = runtime_limit
    # model.Params.MIPGap = 3e-2

    # Decision Variables
    x = model.addVars(vertices, vertices, K, V_d, vtype=GRB.BINARY, name="Routing")
    y = model.addVars(vertices, K, V_d, vtype=GRB.BINARY, name="Assignment")
    z = model.addVars(vertices, vertices)

    ## fixed cost
    FC = gp.quicksum(gp.quicksum(gp.quicksum(F[k] * x[d, i, k, d]
                                             for d in V_d)
                                 for k in K)
                     for i in V_c)
    ## variable cost
    VC = gp.quicksum(gp.quicksum(gp.quicksum(alpha[k] * arcs[i, j] * x[i, j, k, d]
                                             for d in V_d)
                                 for k in K)
                     for (i, j) in arcs)

    # Objective => minimize fixed cost + variable cost
    model.setObjective(VC + FC, GRB.MINIMIZE)

    # Original constraints without additional bounding constraints:

    ## (1) assignment constraint
    model.addConstrs(
        gp.quicksum(gp.quicksum(y[i, k, d]
                                for d in V_d)
                    for k in K) == 1
        for i in V_c
    )

    ## (2) same vehicle same depot
    model.addConstrs(
        y[i, k, d] <= y[d, k, d]
        for i in V_c for k in K for d in V_d
    )

    ## (3) link y and x together
    model.addConstrs(
        gp.quicksum(x[i, j, k, d] for j in vertices) + \
        gp.quicksum(x[j, i, k, d] for j in vertices) == 2 * y[i, k, d]
        for i in V_c for k in K for d in V_d
    )

    ## (4) flow constraint
    model.addConstrs(
        gp.quicksum(x[i, j, k, d] for i in vertices) == gp.quicksum(x[j, i, k, d] for i in vertices)
        for j in vertices for k in K for d in V_d
    )

    ## (5) vehicle assignment constraint
    model.addConstrs(
        y[d, k, d] <= gp.quicksum(x[i, j, k, d] for (i, j) in arcs)
        for k in K for d in V_d
    )

    ## (6) vehicle assignment constraint
    model.addConstrs(
        2 * y[d, k, d] <= gp.quicksum(x[j, d, k, d] for j in V_c) + gp.quicksum(x[d, j, k, d] for j in V_c)
        for k in K for d in V_d
    )

    ## (7) demand satisfaction
    model.addConstrs(
        gp.quicksum(z[i, j] for i in vertices) - gp.quicksum(z[j, i] for i in vertices) == demands[j]
        for j in V_c
    )

    ## (8) total load transported from depots == total demand of all customers
    model.addConstr(
        gp.quicksum(gp.quicksum(z[i, j] for j in V_c) for i in V_d) == gp.quicksum(demands[j] for j in V_c)
    )

    ## (9) capacity constraint
    model.addConstrs(
        z[i, j] <= gp.quicksum(gp.quicksum((Q[k] - demands[i]) * x[i, j, k, d] for d in V_d) for k in K)
        for i in vertices for j in V_c
    )

    # below are the constraints that can improve bounds:
    model.addConstrs(
        z[i, j] >= gp.quicksum(gp.quicksum(demands[j] * x[i, j, k, d]
                                           for d in V_d)
                               for k in K)
        for i in V_c for j in V_c
    )

    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(x[i, j, k, d]
                                            for d in V_d)
                                for k in K)
                    for i in vertices) == 1
        for j in V_c
    )

    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(x[i, j, k, d]
                                            for d in V_d)
                                for k in K)
                    for j in vertices) == 1
        for i in V_c
    )

    model.addConstrs(
        gp.quicksum(x[i, d, k, d] for k in K) <= gp.quicksum(y[i, k, d] for k in K)
        for i in V_c for d in V_d
    )

    model.addConstrs(
        gp.quicksum(x[d, i, k, d] for k in K) <= gp.quicksum(y[i, k, d] for k in K)
        for i in V_c for d in V_d
    )

    model.addConstrs(
        gp.quicksum(x[i, j, k, d] for k in K) + gp.quicksum(y[i, k, d] for k in K) + \
        gp.quicksum(gp.quicksum(y[j, k, h]
                                for h in V_d if h != d)
                    for k in K) <= 2
        for i in V_c for j in V_c if i != j for d in V_d
    )

    model.addConstrs(
        x[i, j, k, d] + x[j, i, k, d] <= 1
        for i in V_c for j in V_c for k in K for d in V_d
    )

    lb_vehicles = math.ceil(sum(demands[i] for i in V_c) / np.amax(Q))
    model.addConstr(
        lb_vehicles <= gp.quicksum(gp.quicksum(gp.quicksum(x[d, i, k, d]
                                                           for d in V_d)
                                               for k in K)
                                   for i in V_c)
    )

    model.optimize()

    # Objective Value:
    objective_value = model.getObjective().getValue()
    if print_out:
        print(f"Objective: {objective_value}")

    # Solution Routes:
    solution = []
    for i, j in arcs:
        for k in K:
            for d in V_d:
                if round(x[i, j, k, d].X) == 1.0:
                    if print_out:
                        print(i, j)
                    solution.append((i, j))

    # Save Runtime for comparison 
    runtime = model.Runtime

    # Save MIP Gap for comparison
    mip_gap = model.MIPGap

    model.dispose()
    return objective_value, solution, runtime, mip_gap


def run_F4(demands, vertices, arcs, V_d, V_c, F, alpha, K, Q, runtime_limit=1800, print_out=False):
    model = gp.Model("Compact formulation with disaggregated loading variables")

    # supress console output from Gurobi
    if not print_out:
        model.Params.LogToConsole = 0

    # Model Parameters(Stopping Condition)
    model.Params.TimeLimit = runtime_limit
    # model.Params.MIPGap = 3e-2

    # Decision Variables
    x = model.addVars(vertices, vertices, K, V_d, vtype=GRB.BINARY, name="Routing")
    y = model.addVars(vertices, K, V_d, vtype=GRB.BINARY, name="Assignment")
    z = model.addVars(vertices, vertices, K)

    ## fixed cost
    FC = gp.quicksum(gp.quicksum(gp.quicksum(F[k] * x[d, i, k, d]
                                             for d in V_d)
                                 for k in K)
                     for i in V_c)
    ## variable cost
    VC = gp.quicksum(gp.quicksum(gp.quicksum(alpha[k] * arcs[i, j] * x[i, j, k, d]
                                             for d in V_d)
                                 for k in K)
                     for (i, j) in arcs)

    # Objective => minimize fixed cost + variable cost
    model.setObjective(VC + FC, GRB.MINIMIZE)

    # Original constraints without additional bounding constraints:

    ## (1) assignment constraint
    model.addConstrs(
        gp.quicksum(gp.quicksum(y[i, k, d]
                                for d in V_d)
                    for k in K) == 1
        for i in V_c
    )

    ## (2) same vehicle same depot
    model.addConstrs(
        y[i, k, d] <= y[d, k, d]
        for i in V_c for k in K for d in V_d
    )

    ## (3) link y and x together
    model.addConstrs(
        gp.quicksum(x[i, j, k, d] for j in vertices) + \
        gp.quicksum(x[j, i, k, d] for j in vertices) == 2 * y[i, k, d]
        for i in V_c for k in K for d in V_d
    )

    ## (4) flow constraint
    model.addConstrs(
        gp.quicksum(x[i, j, k, d] for i in vertices) == gp.quicksum(x[j, i, k, d] for i in vertices)
        for j in vertices for k in K for d in V_d
    )

    ## (5) vehicle assignment constraint
    model.addConstrs(
        y[d, k, d] <= gp.quicksum(x[i, j, k, d] for (i, j) in arcs)
        for k in K for d in V_d
    )

    ## (6) vehicle assignment constraint
    model.addConstrs(
        2 * y[d, k, d] <= gp.quicksum(x[j, d, k, d] for j in V_c) + gp.quicksum(x[d, j, k, d] for j in V_c)
        for k in K for d in V_d
    )

    ## (10) total load transported from depots == total demand of all customers
    model.addConstr(
        gp.quicksum(gp.quicksum(gp.quicksum(z[i, j, k]
                                            for k in K)
                                for j in V_c)
                    for i in V_d) ==
        gp.quicksum(demands[j] for j in V_c)
    )

    ## (11) customer demand satisfaction
    model.addConstrs(
        gp.quicksum(z[i, j, k] for i in vertices) - gp.quicksum(z[j, i, k] for i in vertices) ==
        gp.quicksum(demands[j] * y[j, k, d] for d in V_d)
        for j in V_c for k in K
    )

    ## (12) capacity constraint
    model.addConstrs(
        z[i, j, k] <= gp.quicksum((Q[k] - demands[i]) * x[i, j, k, d] for d in V_d)
        for i in vertices for j in V_c for k in K
    )

    # below are the constraints that can improve bounds:
    model.addConstrs(
        z[i, j, k] >= gp.quicksum(demands[j] * x[i, j, k, d]
                                  for d in V_d)
        for i in V_c for j in V_c for k in K
    )

    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(x[i, j, k, d]
                                            for d in V_d)
                                for k in K)
                    for i in vertices) == 1
        for j in V_c
    )

    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(x[i, j, k, d]
                                            for d in V_d)
                                for k in K)
                    for j in vertices) == 1
        for i in V_c
    )

    model.addConstrs(
        gp.quicksum(x[i, d, k, d] for k in K) <= gp.quicksum(y[i, k, d] for k in K)
        for i in V_c for d in V_d
    )

    model.addConstrs(
        gp.quicksum(x[d, i, k, d] for k in K) <= gp.quicksum(y[i, k, d] for k in K)
        for i in V_c for d in V_d
    )

    model.addConstrs(
        gp.quicksum(x[i, j, k, d] for k in K) + gp.quicksum(y[i, k, d] for k in K) + gp.quicksum(gp.quicksum(y[j, k, h]
                                                                                                             for h in
                                                                                                             V_d if
                                                                                                             h != d)
                                                                                                 for k in K) <= 2
        for i in V_c for j in V_c if i != j for d in V_d
    )

    model.addConstrs(
        x[i, j, k, d] + x[j, i, k, d] <= 1
        for i in V_c for j in V_c for k in K for d in V_d
    )

    lb_vehicles = math.ceil(sum(demands[i] for i in V_c) / np.amax(Q))
    model.addConstr(
        lb_vehicles <= gp.quicksum(gp.quicksum(gp.quicksum(x[d, i, k, d]
                                                           for d in V_d)
                                               for k in K)
                                   for i in V_c)
    )

    model.optimize()

    # Objective Value:
    objective_value = model.getObjective().getValue()
    if print_out:
        print(f"Objective: {objective_value}")

    # Solution Routes:
    solution = []
    for i, j in arcs:
        for k in K:
            for d in V_d:
                if round(x[i, j, k, d].X) == 1.0:
                    if print_out:
                        print(i, j)
                    solution.append((i, j))

    # Save Runtime for comparison 
    runtime = model.Runtime

    # Save MIP Gap for comparison
    mip_gap = model.MIPGap

    model.dispose()
    return objective_value, solution, runtime, mip_gap


def run_F5(demands, vertices, arcs, V_d, V_c, F, alpha, K, Q, runtime_limit=1800, print_out=False):
    model = gp.Model("Capacity-indexed formulation")

    # supress console output from Gurobi
    if not print_out:
        model.Params.LogToConsole = 0

    # Model Parameters(Stopping Condition)
    model.Params.TimeLimit = runtime_limit
    # model.Params.MIPGap = 3e-2

    # Decision Variables
    x = {}
    for i in vertices:
        for j in vertices:
            for k in K:
                for d in V_d:
                    for q in range(0, Q[k] + 1):
                        x[i, j, k, d, q] = model.addVar(vtype=GRB.BINARY, name="Routing")

    ## fixed cost
    FC = gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(F[k] * x[d, i, k, d, q]
                                                         for q in range(1, Q[k] + 1))
                                             for d in V_d)
                                 for k in K)
                     for i in V_c)
    ## variable cost
    VC = gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(alpha[k] * arcs[i, j] * x[i, j, k, d, q]
                                                         for q in range(0, Q[k] + 1))
                                             for d in V_d)
                                 for k in K)
                     for (i, j) in arcs)

    # Objective => minimize fixed cost + variable cost
    model.setObjective(VC + FC, GRB.MINIMIZE)

    # Original constraints without additional bounding constraints:

    ## (13) assignment constraint
    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(x[j, i, k, d, q]
                                                        for q in range(1, Q[k] + 1))
                                            for d in V_d)
                                for k in K)
                    for j in vertices) == 1
        for i in V_c
    )

    ## (14) flow conservation aand assigned vehicle number constraint
    model.addConstrs(
        gp.quicksum(gp.quicksum(x[d, i, k, d, q]
                                for q in range(1, Q[k] + 1))
                    for i in V_c) ==
        gp.quicksum(x[i, d, k, d, 0] for i in V_c)
        for k in K for d in V_d
    )

    ## (15) vehicle capacity requirement constraint
    model.addConstrs(
        gp.quicksum(x[j, i, k, d, q] for j in vertices) ==
        gp.quicksum(x[i, j, k, d, (q - demands[i])] for j in vertices)
        for i in V_c for k in K for d in V_d for q in range(demands[i], Q[k] + 1)
    )

    # below are the constraints that can improve bounds:
    model.addConstrs(
        x[i, j, k, d, q] == 0
        for i in vertices for j in V_c for k in K for d in V_d for q in range(Q[k] - demands[i] + 1, Q[k] + 1)
    )

    lb_vehicles = math.ceil(sum(demands[i] for i in V_c) / np.amax(Q))
    model.addConstr(
        lb_vehicles <= gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(x[d, i, k, d, q]
                                                                       for q in range(demands[i], Q[k] + 1))
                                                           for d in V_d)
                                               for k in K)
                                   for i in V_c)
    )

    model.addConstr(
        gp.quicksum(demands[i] for i in V_c) <=
        gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(q * x[i, j, k, d, q]
                                                                    for q in range(demands[i], Q[k] + 1))
                                                        for d in V_d)
                                            for k in K)
                                for j in V_c)
                    for i in vertices)
    )

    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(q * x[j, i, k, d, q]
                                                        for q in range(demands[i], Q[k] - demands[j] + 1))
                                            for d in V_d)
                                for k in K)
                    for j in vertices) - \
        gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(q * x[i, j, k, d, q]
                                                        for q in range(demands[j], Q[k] - demands[i] + 1))
                                            for d in V_d)
                                for k in K)
                    for j in vertices) == demands[i]
        for i in V_c
    )

    model.addConstrs(
        gp.quicksum(x[i, j, k, d, q] for q in range(demands[j], Q[k] - demands[i] + 1)) <=
        gp.quicksum(x[h, d, k, d, 0] for h in V_c)
        for i in V_c for j in V_c for k in K for d in V_d
    )

    model.addConstrs(
        x[h, d, k, d, 0] <= gp.quicksum(gp.quicksum(gp.quicksum(x[i, j, k, d, q]
                                                                for q in range(demands[j], Q[k] - demands[i] + 1))
                                                    for j in V_c)
                                        for i in vertices)
        for h in V_c for k in K for d in V_d
    )

    model.addConstrs(
        gp.quicksum(gp.quicksum(gp.quicksum(gp.quicksum(x[i, j, k, d, q]
                                                        for q in range(0, Q[k] + 1))
                                            for d in V_d)
                                for k in K)
                    for j in vertices) == 1
        for i in V_c
    )

    model.optimize()

    # Objective Value:
    objective_value = model.getObjective().getValue()
    if print_out:
        print(f"Objective: {objective_value}")

    # Solution Routes:
    solution = []
    for i, j in arcs:
        for k in K:
            for d in V_d:
                for q in range(0, Q[k] + 1):
                    if round(x[i, j, k, d, q].X) == 1.0:
                        if print_out:
                            print(i, j, q)
                        solution.append((i, j))

    # Save Runtime for comparison 
    runtime = model.Runtime

    # Save MIP Gap for comparison
    mip_gap = model.MIPGap

    model.dispose()
    return objective_value, solution, runtime, mip_gap
