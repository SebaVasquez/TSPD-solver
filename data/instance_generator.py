# -*- coding: utf-8 -*-
"""
Created on Sun Sep 16 09:31:34 2018

@author: savas
"""

from classes.node import Node

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

def run(n, alpha, seed=0):
    np.random.seed(seed)
        
    pos = dict(zip([i + 1 for i in range(n)], list(map(tuple, np.random.random([n, 2]) * 100))))
    pos[n + 1] = pos[1]
    nodes = [Node(i, pos[i][0], pos[i][1]) for i in range(1, n + 1)] + [Node(n + 1, pos[n + 1][0], pos[n + 1][1])]
    truck_travel_time = {
        (i, j): int(np.sqrt((i.x - j.x) ** 2 + (i.y - j.y) ** 2) * 10)
        for i in nodes for j in nodes if i != j
    }
    drone_travel_time = {
        a: truck_travel_time[a] / alpha 
        for a in truck_travel_time
    }
    
    return nodes, truck_travel_time, drone_travel_time

if __name__ == '__main__':
    n = 10
    pos, c, d = run(n, 2, 1)
    nodes = [i + 1 for i in range(n)]
    
    arcs = [(i, i + 1) for i in range(1, n)]
    labels = {i: str(i) for i in nodes}

    G = nx.Graph()
    nx.draw_networkx_nodes(G, pos, nodes, node_color='r', node_size=170, alpha=1)
    nx.draw_networkx_nodes(G, pos, nodes, node_color='w', node_size=140, alpha=1)
    nx.draw_networkx_labels(G, pos, labels, font_size=7)
    nx.draw_networkx_edges(G, pos, arcs)
    plt.axis('off')
    plt.show()

    

