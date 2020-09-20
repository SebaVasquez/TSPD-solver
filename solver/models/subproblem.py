# -*- coding: utf-8 -*-
"""
Created on Wed Jan 16 13:36:37 2019

@author: savas
"""

from gurobipy import GRB, quicksum

from classes.model import Model

class SubProblem(Model):
    def __init__(self, instance, params):
        super().__init__(instance)
        self.M = params['M']
        self._init_model()

    def set_constr_rhs(self, x_recourse):
        for a in self.constr:
            self.constr[a].RHS = x_recourse[a]
    
    def _solve(self):
        self.model.optimize()

    @property
    def objval(self):
        return self.model.objval
    
    @property
    def runtime(self):
        return self.model.runtime
    
    @property
    def status(self):
        return self.model.status

    def _init_model(self):
        model = self.model
        model.ModelName = 'SubProblem'
        delta_in = self.instance.delta_in
        delta_out = self.instance.delta_out
        delta_in_operation = self.instance.delta_in_operation
        delta_out_operation = self.instance.delta_out_operation
        depot_1 = self.instance.depot_1
        depot_2 = self.instance.depot_2
        N = self.instance.nodes
        n = self.instance.n
        A = self.instance.arcs
        c = self.instance.truck_travel_time
        d = self.instance.drone_travel_time
        O = self.instance.operations
        customer_nodes = [i for i in N if i not in [depot_1, depot_2]]
        M = self.M
        
        model.Params.InfUnbdInfo = 0
        model.Params.OutputFlag = 0

        u = model.addVars(N, vtype=GRB.CONTINUOUS, name='u')
        y = model.addVars(N, N, vtype=GRB.CONTINUOUS, name='y')
        z = model.addVars(A, vtype=GRB.CONTINUOUS, lb=-GRB.INFINITY, name='z')
        v = model.addVars(N, vtype=GRB.CONTINUOUS, name='v')
        gamma = model.addVars(N, vtype=GRB.CONTINUOUS, name='gamma')
        c_hat = model.addVars(A, vtype=GRB.CONTINUOUS, name='c_bar')
        o = model.addVars(O, vtype=GRB.BINARY, name='o')
        f = model.addVars(A, vtype=GRB.BINARY, name='f')
        w = model.addVars(O, vtype=GRB.CONTINUOUS, name='w')  

        self.variables = {
            'f': f,
            'o': o,
            'w': w
        }

        # Constraints
        self.constr = model.addConstrs(z[a] == 0 for a in A)

        # Truck route
        model.addConstr(u[depot_1] == 1)
        model.addConstr(u[depot_2] == quicksum(gamma[j] for j in N))
        model.addConstrs(u[i] - u[j] + 1 <= n * (1 - z[i, j]) for (i, j) in A if j != depot_1)
        model.addConstrs(u[i] <= (n - 1) * gamma[i] for i in customer_nodes)
        model.addConstrs(2 * gamma[i] <= u[i] for i in customer_nodes)
        
        model.addConstrs(quicksum(y[i, j] for i in N if i != j) == u[j] - gamma[j] for j in N if j != depot_2)
        model.addConstrs(y[i, j] + y[j, i] <= gamma[i] for i in N for j in N if i != j)
        model.addConstrs(y[i, j] + y[j, i] <= gamma[j] for i in N for j in N if i != j)
        model.addConstrs(gamma[i] + gamma[j] - 1 <= y[i, j] + y[j, i]  for i in N for j in N if i != j)
        
        model.addConstrs(gamma[i] == quicksum(z[a] for a in delta_out[i]) for i in N)

        # Truck route costs
        model.addConstr(v[depot_1] == 0)
        model.addConstr(v[depot_2] == quicksum(z[a] * c[a] for a in A))
        model.addConstrs(
            v[i] - v[j] + c[i, j] <= M * (1 - z[i, j]) 
            for (i, j) in A if j != depot_1 and (i, j) != (depot_1, depot_2)
        )
        
        model.addConstrs(
            v[i] <= quicksum(z[a] * c[a] for a in A) - 
            quicksum(z[a] * c[a] for a in delta_in[depot_2]) 
            for i in customer_nodes
        )

        model.addConstrs(c_hat[i, j] <= v[j] - v[i] + M * (1 - y[i, j]) for i in N for j in N if i != j)
        model.addConstrs(c_hat[i, j] <= M * y[i, j] for i in N for j in N if i != j)

        # Drone route
        model.addConstrs(f[a] <= z[a] for a in A)
        model.addConstrs(quicksum(o[i, k, j] for k in N if (i, k, j) in O) <= y[i, j] for i in N for j in N if i != j)

        model.addConstrs(quicksum(o[e] for e in delta_out_operation[i]) <= (n - 2) * gamma[i] for i in N)
        model.addConstrs(quicksum(o[e] for e in delta_in_operation[i]) <= (n - 2) * gamma[i] for i in N)

        model.addConstrs(quicksum(o[i, k, j] for i in N for j in N if (i, k, j) in O) == 1 - gamma[k] for k in N)
        
        model.addConstrs(
            quicksum(o[e] for e in delta_out_operation[i] if e[0] != e[2]) +
            quicksum(f[e] for e in delta_out[i]) - 
            quicksum(o[e] for e in delta_in_operation[i] if e[0] != e[2]) - 
            quicksum(f[e] for e in delta_in[i])
            == 0 for i in customer_nodes
        )
        model.addConstrs(
            o[i, k, i] <= quicksum(o[e] for e in delta_in_operation[i] if e[0] != e[2])
            + quicksum(f[e] for e in delta_in[i]) 
            for i in N for k in N if (i, k, i) in O and i != depot_1
        )

        model.addConstr(
            quicksum(o[e] for e in delta_out_operation[depot_1]) 
            + quicksum(f[e] for e in delta_out[depot_1]) >= 1
        )
        model.addConstr(
            quicksum(o[e] for e in delta_in_operation[depot_2]) 
            + quicksum(f[e] for e in delta_in[depot_2]) == 1
        )
        
        model.addConstrs(
            w[i, k, j] >= o[i, k, j] * (d[i, k] + d[k, j]) - c_hat[i, j] 
            for (i, k, j) in O if i != j
        )
        model.addConstrs(
            w[i, k, j] >= o[i, k, j] * (d[i, k] + d[k, j]) 
            for (i, k, j) in O if i == j
        )
        
        # Objective
        obj = obj = quicksum(w[e] for e in O)
        model.setObjective(obj, GRB.MINIMIZE) 