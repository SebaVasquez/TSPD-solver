
from gurobipy import GRB, quicksum

from classes.model import Model

class MasterProblem(Model):
    def __init__(self, instance, params):
        super().__init__(instance)
        self.time_limit = params.get('time_limit')
        self.SEC = params.get('SEC')
        self.DB = params.get('activate_DB')
        self.min_visited_nodes = params.get('min_visited_nodes')
        self._init_model()

    def _solve(self, callback):
        self.model.optimize(callback)
    
    def reset(self):
        self._reset()
        self._init_model()

    def _init_model(self):
        def _define_SEC():
            SEC = self.SEC

            if SEC == 'OCF':
                f = model.addVars(A, vtype=GRB.INTEGER, name='f')

                model.addConstrs(f[a] <= n * x[a] for a in A)
                model.addConstrs(
                    quicksum(f[a] for a in delta_out[i]) - quicksum(f[a] for a in delta_in[i]) ==
                    - gamma[i] for i in N if i != depot_2
                )
                model.addConstr(
                    quicksum(f[a] for a in delta_out[depot_2]) == 
                    quicksum(gamma[i] for i in N if i != depot_2)
                )
                model.addConstr(quicksum(f[a] for a in delta_in[depot_2]) == 0)

                return f
            
            if SEC == 'MCF':
                f_m = model.addVars(A, [i for i in N if i != depot_2], vtype=GRB.CONTINUOUS, name='f_m')

                model.addConstrs(
                    f_m[i, j, k] <= x[i, j] 
                    for (i, j) in A for k in N if k != depot_2
                )
                model.addConstrs(
                    quicksum(f_m[i, j, k] for (i, j) in delta_out[depot_2]) == gamma[k] 
                    for k in N if k != depot_2
                )
                model.addConstrs(quicksum(f_m[i, j, k] for (i, j) in delta_in[depot_2]) == 0 
                for k in N if k != depot_2)
                model.addConstrs(
                    quicksum(f_m[e[0], e[1], k] for e in delta_out[i]) -
                    quicksum(f_m[e[0], e[1], k] for e in delta_in[i]) == 0 
                    for k in N for i in N
                    if k != depot_2 and i != depot_2 and i != k
                )
                model.addConstrs(
                    quicksum(f_m[e[0], e[1], k] for e in delta_out[k]) -
                    quicksum(f_m[e[0], e[1], k] for e in delta_in[k]) == 
                    - gamma[k] 
                    for k in N if k != depot_2
                )

                return f_m

        model = self.model
        model.ModelName = 'MasterProblem'
        delta_in = self.instance.delta_in
        delta_out = self.instance.delta_out
        depot_1 = self.instance.depot_1
        depot_2 = self.instance.depot_2
        N = self.instance.nodes
        n = self.instance.n
        A = self.instance.arcs
        c = self.instance.truck_travel_time
        d = self.instance.drone_travel_time
        min_visited_nodes = self.min_visited_nodes
        customer_nodes = [i for i in N if i not in [depot_1, depot_2]]

        model.params.LazyConstraints = 1
        if self.time_limit:
            model.params.TimeLimit = self.time_limit

        # Variables
        x = model.addVars(A, vtype=GRB.BINARY, name='x')
        gamma = model.addVars(N, vtype=GRB.CONTINUOUS, ub=1, name='gamma')
        theta = model.addVar(vtype=GRB.CONTINUOUS, name='theta')
        f = _define_SEC()

        self.variables = {
            'x': x,
            'gamma': gamma,
            'theta': theta,
            'f': None if not f else f,
        }

        # Constraints
        model.addConstrs(quicksum(x[a] for a in delta_in[i]) == gamma[i] for i in N)
        model.addConstrs(quicksum(x[a] for a in delta_out[i]) == gamma[i] for i in N)
        model.addConstr(x[depot_2, depot_1] == 1)

        model.addConstrs(x[i, j] <= quicksum(x[i_2, j_2] for (i_2, j_2) in delta_in[depot_2] if i_2.id <= j.id)  
                     for (i, j) in delta_out[depot_1])
        model.addConstrs(x[i, j] <= quicksum(x[i_2, j_2] for (i_2, j_2) in delta_out[depot_1] if i.id <= j_2.id)  
                        for (i, j) in delta_in[depot_2])
        model.addConstr(quicksum(gamma[i] for i in customer_nodes) >= min_visited_nodes)
        model.addConstr(quicksum(1 - gamma[i] for i in customer_nodes) >= 1)

        if self.DB:
            l = model.addVars(A, vtype=GRB.BINARY)
            u = model.addVars(A, vtype=GRB.BINARY)

            self.variables['l'] = l
            self.variables['u'] = u

            model.addConstrs(quicksum(u[e] for e in delta_out[i]) == 1 - gamma[i] for i in customer_nodes)
            model.addConstrs(quicksum(l[e] for e in delta_in[i]) == 1 - gamma[i] for i in customer_nodes)
            model.addConstrs(quicksum(u[e] for e in delta_in[i]) == quicksum(l[e] for e in delta_out[i]) for i in customer_nodes)
            model.addConstrs(quicksum(l[e] for e in delta_out[i]) <= (n - 2) * gamma[i] for i in N if i != depot_2)
            model.addConstrs(quicksum(u[e] for e in delta_in[i]) <= (n - 2) * gamma[i] for i in N if i != depot_2)
            model.addConstr(quicksum(l[e] for e in delta_out[depot_1]) >= 1)
            model.addConstr(quicksum(u[e] for e in delta_in[depot_2]) == 0)
            
            loop = model.addVars(A, vtype=GRB.BINARY)
            model.addConstrs(loop[i, k] >= l[i, k] + u[k, i] - 1 for (i, k) in A if i != depot_2)
            model.addConstrs(loop[i, k] <= l[i, k] for (i, k) in A if i != depot_2)
            model.addConstrs(loop[i, k] <= u[k, i] for (i, k) in A if i != depot_2)
            model.addConstrs(
                loop[e_2] <= quicksum(l[e_1] - loop[e_1] for e_1 in delta_out[i]) 
                for i in N for e_2 in delta_out[i] if i != depot_2
            )
            
            s = {(i, j): l[i, j] + u[i, j] - loop[i, j] - loop[j, i] for (i, j) in A}
            model.addConstrs(
                s[i, j] <= quicksum(s[i_2, j_2] for (i_2, j_2) in delta_in[depot_1] if i_2.id <= j.id) 
                for (i, j) in delta_out[depot_1]
            )
            model.addConstrs(
                s[i, j] <= quicksum(s[i_2, j_2] for (i_2, j_2) in delta_out[depot_1] if i.id <= j_2.id) 
                for (i, j) in delta_in[depot_1]
            )

            LB = quicksum(d[e] * (l[e] + u[e]) - c[e] * x[e] for e in A)

            model.addConstr(theta >= LB)

        obj = quicksum(x[e] * c[e] for e in A) + theta
        model.setObjective(obj, GRB.MINIMIZE)