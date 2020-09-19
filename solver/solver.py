
from classes.solution import Solution
from functions import plotter 
from solver.models.master_problem import MasterProblem
from solver.models.subproblem import SubProblem
from solver.callback import Callback
from solver.functions.solver_functions import get_route

from time import time

def solve(instance):
    master_problem_params = {
        'SEC': None,
        'activate_DB': False,
        'min_visited_nodes': instance.n // 2,
        'time_limit': 60000000
    }
    subproblem_params = {
        'M': sum(sorted(instance.truck_travel_time.values())[::-1][:2 * (instance.n - 1):2])
    }

    functions = {
        'get_route': get_route
    }

    master_problem = MasterProblem(instance, master_problem_params)
    subproblem = SubProblem(instance, subproblem_params)
    callback = Callback(instance, functions)
    solution = Solution(instance)

    cb = callback.callback(master_problem, subproblem, solution)
    
    solution.start_time = time()
    master_problem._solve(cb)
    
    solution.truck_arcs = [a for a in instance.arcs if master_problem.variables['x'][a].X > 0.5]
    recourse = {a: 0 for a in instance.arcs}
    for a in solution.truck_arcs:
        recourse[a] = 1
    subproblem.set_constr_rhs(recourse)
    subproblem._solve()
    solution.operations = [e for e in instance.operations if subproblem.variables['o'][e].X > 0.5]

    plotter.plot(instance, solution.truck_arcs, solution.drone_arcs)


