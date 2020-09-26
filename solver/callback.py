
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
        self.get_route_recourse = functions['get_route_recourse']
        self.get_shortcuts = functions['get_shortcuts']
        self.graph = DiGraph()

    def callback(self, master_problem, subproblem, solution):
        N = self.instance.nodes
        customer_nodes = [node for node in N if node not in [self.instance.depot_1, self.instance.depot_2]]
        A = self.instance.arcs
        depot = self.instance.depot_1
        delta_out = self.instance.delta_out
        delta_out_S = self.instance.delta_out_S
        get_route = self.get_route
        get_route_recourse = self.get_route_recourse
        get_shortcuts = self.get_shortcuts
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

            def feasibility_cut_function(recourse, solution):
                '''
                This function generates and adds an feasibility cut

                Args: Variables as recourse, solution (class)
                '''
                
                x_arcs = recourse['x_arcs']
                gamma_recourse = recourse['gamma']
                truck_route = recourse['truck_route']
                max_drop = sum([gamma_recourse[i] for i in customer_nodes]) - min_visited_nodes

                # Shortcuts generation
                shortcuts = get_shortcuts(truck_route, max_drop)

                delta = quicksum((1 - x[a]) for a in x_arcs)\
                        - quicksum(x[a] for a in shortcuts)\
                        - quicksum((1 - gamma[i]) for i in N if gamma_recourse[i] == 1)

                m.cbLazy(delta >= 1)
                solution.feasibility_cuts += 1
            
            def optimality_cut_function(recourse, waiting, solution):
                '''
                This function generates and adds an IOC

                Args: Variables as recourse, Q(x_recourse), solution (class)
                '''

                x_arcs = recourse['x_arcs']
                gamma_recourse = recourse['gamma']
                theta_recourse = recourse['theta']
                truck_route = recourse['truck_route']
                max_drop = sum([gamma_recourse[i] for i in customer_nodes]) - min_visited_nodes
                
                # Shortcuts generation
                shortcuts = get_shortcuts(truck_route, max_drop)
                    
                # Optimality cut
                if theta_recourse <= waiting - 1e-8:
                    delta = quicksum((1 - x[a]) for a in x_arcs)\
                            - quicksum(x[a] for a in shortcuts)\
                            - quicksum((1 - gamma[i]) for i in N if gamma_recourse[i] == 1)
                    m.cbLazy(theta >= - waiting * delta + waiting )
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
                        elif master_problem.SEC == 'OCF' or not master_problem.SEC:
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
                x_recourse = get_route_recourse(x_arcs, A)
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
                
                truck_route = get_route(x_arcs, depot)
                recourse['truck_route'] = truck_route
                # Optimality cut check
                if truck_route in solution.Q:  # Truck route is already tracked
                    waiting = solution.Q[truck_route]
                else:
                    set_constr_rhs(x_recourse)
                    subproblem._solve()
                    solution.IOC_time_list.append(subproblem.runtime)
                    
                    # Feasibility cut
                    if subproblem.status != 2:
                        feasibility_cut_function(recourse, solution)
                        return
                    
                    waiting = subproblem.objval
                    solution.Q[truck_route] = waiting  # Track current truck route
                        
                if waiting > 1e-8:
                    optimality_cut_function(recourse, waiting, solution)  # Add optimality cut
                
                # Incumbent achievement time
                if theta_recourse >= waiting - 1e-8:
                    solution.incumbent_time = time() - solution.start_time

        return _run_callback
