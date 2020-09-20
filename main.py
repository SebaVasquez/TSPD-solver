
import sys

from classes.instance import Instance
from solver.solver import solve
from data import instance_generator as ins_gen

if len(sys.argv) != 5:
    sys.exit('Incorrect number of params. Please execute solver as \'python main.py <n> <alpha> <L> <instance_number>\'')
n, alpha, L, seed = list(map(int, sys.argv[1:]))

nodes, c, d = ins_gen.run(n, alpha, seed=seed)
instance = Instance(1, nodes, alpha, L, c, d)

solve(instance)

print()

