
import numpy as np

class Instance:
    def __init__(self, instance_id, nodes, alpha, L, truck_costs=dict(), drone_costs=dict()):
        self.id = instance_id
        self.n = len(nodes)
        self.alpha = alpha
        self.L = L
        self.nodes = nodes
        self.arcs = [(i, j) for i in nodes for j in nodes if i != j]
        self.operations = [
            (i, k, j) for i in nodes for k in nodes for j in nodes
            if k.id not in [1, i.id, j.id, len(nodes)] 
            and drone_costs[i, k] + drone_costs[k, j] <= L
        ]
        self.truck_travel_time = truck_costs
        self.drone_travel_time = drone_costs
        self.delta_in = {i: [a for a in self.arcs if a[1] == i] for i in nodes}
        self.delta_out = {i: [a for a in self.arcs if a[0] == i] for i in nodes}
        self.delta_in_operation = {i: [o for o in self.operations if o[2] == i] for i in nodes}
        self.delta_out_operation = {i: [o for o in self.operations if o[0] == i] for i in nodes}
        for node in nodes:
            if node.id == 1:
                self.depot_1 = node
            if node.id == self.n:
                self.depot_2 = node
        self.min_visited_nodes = 0

    def delta_in_S(self, S):
        return [(i, j) for (i, j) in self.arcs if j in S and i not in S]
    
    def delta_out_S(self, S):
        return [(i, j) for (i, j) in self.arcs if i in S and j not in S]
    
    def compute_travel_times(self):
        self.truck_travel_time = {
            (i, j): int(np.sqrt((i.x - j.x) ** 2 + (i.y - j.y) ** 2) * 10) / 10
            for i in self.nodes for j in self.nodes if i != j
        }
        self.drone_travel_time = {
            (i, j): self.truck_travel_time[i, j] / self.alpha 
            for (i, j) in self.truck_travel_time
        }

    def __repr__(self):
        return str(self.id)