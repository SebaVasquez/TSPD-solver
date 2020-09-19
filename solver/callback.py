
from gurobipy import GRB, quicksum
from networkx import DiGraph
from time import time
import sys

from classes.instance import Instance
from solver.functions.solver_functions import get_disconnected_components, get_disconnected_component_mincut

class Callback:
    def __init__(self, instance, functions):
        self.instance = instance
        self.get_route = functions['get_route']
        self.graph = DiGraph()
    
    def callback(self, master_problem, subproblem, solution):
        N = self.instance.nodes
        A = self.instance.arcs
        depot = self.instance.depot_1
        delta_out = self.instance.delta_out
        delta_out_S = self.instance.delta_out_S
        get_route = self.get_route
        graph = self.graph

        x = master_problem.variables['x']
        gamma = master_problem.variables['gamma']
        theta = master_problem.variables['theta']

        min_visited_nodes = master_problem.min_visited_nodes
        set_constr_rhs = subproblem.set_constr_rhs

        def _run_callback(m, where):
            def get_truck_route(model):
                # Retrieves x
                x_arcs = list()
                for a in A:
                    arc = model.cbGetSolution([x[a]])
                    if arc[0] > 0.5:
                        x_arcs.append(a)
                return x_arcs  
            
            def optimality_cut_function(recourse, waiting, solution):
                '''
                This function generates and add an IOC

                Args: Variables as recourse, Q(x_recourse), solution (class)
                '''

                x_arcs = recourse['x_arcs']
                gamma_recourse = recourse['gamma']
                theta_recourse = recourse['theta']
                truck_route = recourse['truck_route']
                max_drop = sum([gamma_recourse[i] for i in N]) - min_visited_nodes
                
                # Shortcuts generation
                shortcuts = list()
                if max_drop > 0:
                    shortcuts = [
                        (truck_route[i], truck_route[j]) for i in range(len(truck_route) - 2) 
                        for j in range(i + 2, len(truck_route)) if j <= i + 1 + max_drop
                    ]
                    
                # Optimality cut
                if theta_recourse <= waiting - 1e-8:
                    m.cbLazy(
                        theta >= - waiting * (
                            quicksum((1 - x[a]) for a in x_arcs) - 
                            quicksum(x[a] for a in shortcuts) - 
                            quicksum((1 - gamma[i]) for i in N if gamma_recourse[i] == 1)
                        )
                        + waiting
                    )
                    solution.IOC += 1
                            
            # Callback execution
            if where == GRB.Callback.MIPNODE:  # Fractional solution
                depth = m.cbGet(GRB.Callback.MIPNODE_NODCNT)

                if depth == 0:
                    # Variables retrieval
                    recourse = {
                        'x': m.cbGetNodeRel(x),
                        'gamma': m.cbGetNodeRel(gamma)
                    }

                    # SEC check
                    if not master_problem.SEC:
                        t_0 = time()
                        disconnected_components = get_disconnected_components(graph, depot, recourse)
                        if disconnected_components:
                            solution.disconnected_components_time_list.append(time() - t_0)
                            for component in disconnected_components:
                                for node in component:
                                    m.cbLazy(quicksum(x[a] for a in delta_out_S(component)) >= gamma[node])
                                    solution.disconnected_components += 1
                            return
                        else:
                            t_0 = time()
                            disconnected_component = get_disconnected_component_mincut(graph, N, recourse)
                            if disconnected_component:
                                solution.disconnected_components_mincut_time_list.append(time() - t_0)
                                m.cbLazy(
                                    quicksum(x[a] for a in delta_out_S(disconnected_component[2])) 
                                    >= gamma[disconnected_component[1]]
                                )
                                solution.disconnected_components_mincut += 1
                                return

            if where == GRB.Callback.MIPSOL:  # Integer solution  
                solution.integer_solutions += 1

                # Variables retrieval
                x_recourse = {a: 0 for a in A}
                x_arcs = get_truck_route(m)
                for a in x_arcs:
                    x_recourse[a] = 1
                gamma_recourse = {
                    i: sum([x_recourse[a] for a in delta_out[i]]) 
                    for i in N
                }
                theta_recourse = m.cbGetSolution(theta)
                recourse = {
                    'x': x_recourse,
                    'x_arcs': x_arcs,
                    'gamma': gamma_recourse,
                    'theta': theta_recourse
                }

                # SEC check
                if not master_problem.SEC:
                    t_0 = time()
                    disconnected_components = get_disconnected_components(graph, depot, recourse)
                    if disconnected_components:
                        solution.disconnected_components_time_list.append(time() - t_0)
                        for component in disconnected_components:
                            for node in component:
                                m.cbLazy(quicksum(x[a] for a in delta_out_S(component)) >= gamma[node])
                                solution.disconnected_components += 1
                        return

                # Optimality cut check
                truck_route = get_route(x_arcs, depot)
                if truck_route in solution.Q:  # Truck route is already tracked
                    waiting = solution.Q[truck_route]
                else:
                    set_constr_rhs(x_recourse)
                    subproblem._solve()
                    solution.IOC_time_list.append(subproblem.runtime)
                    if subproblem.status != 2:
                        print(x_arcs)
                        sys.exit('\nSubproblem is infeasible. Dropping out!\n')
                    waiting = subproblem.objval
                    solution.Q[truck_route] = waiting  # Track current truck route
                        
                if waiting > 1e-8:
                    recourse['truck_route'] = truck_route
                    optimality_cut_function(recourse, waiting, solution)  # Add optimality cut
                
                # Incumbent achievement time
                if theta_recourse >= waiting - 1e-8:
                    solution.incumbent_time = time() - solution.start_time

        return _run_callback