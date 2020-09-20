
from classes.solution import Solution
from functions import plotter 
from solver.models.master_problem import MasterProblem
from solver.models.subproblem import SubProblem
from solver.callback import Callback
from solver.functions.solver_functions import get_route, get_route_recourse, get_shortcuts

from time import time

def solve(instance):
    master_problem_params = {
        'SEC': None,
        'activate_DB': False,
        'min_visited_nodes': 0,
        'time_limit': 3600
    }
    subproblem_params = {
        'M': sum(sorted(instance.truck_travel_time.values())[::-1][:2 * (instance.n - 1):2])
    }

    functions = {
        'get_route': get_route,
        'get_route_recourse': get_route_recourse,
        'get_shortcuts': get_shortcuts
    }

    master_problem = MasterProblem(instance, master_problem_params)
    subproblem = SubProblem(instance, subproblem_params)
    callback = Callback(instance, functions)
    solution = Solution(instance)

    cb = callback.callback(master_problem, subproblem, solution)
    
    solution.start_time = time()
    master_problem._solve(cb)
    
    solution.truck_arcs = [a for a in instance.arcs if master_problem.variables['x'][a].X > 0.5]
    recourse = get_route_recourse(solution.truck_arcs, instance.arcs)

    subproblem.set_constr_rhs(recourse)
    subproblem._solve()
    solution.operations = [e for e in instance.operations if subproblem.variables['o'][e].X > 0.5]

    plotter.plot(instance, solution.truck_arcs, solution.drone_arcs)


