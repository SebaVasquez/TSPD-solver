
from classes.instance import Instance
from solver.solver import solve
from data import instance_generator as ins_gen

n = 8
alpha = 2
L = float('inf')
seed = 0
nodes, c, d = ins_gen.run(n, alpha, seed=seed)
instance = Instance(1, nodes, alpha, L, c, d)

solve(instance)

print()

