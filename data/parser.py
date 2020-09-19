
from os import listdir
from os.path import isfile, join
import pandas as pd

from classes.instance import Instance
from classes.node import Node

def parse_agatz(n):
    path = './data/instances_agatz/uniform'
    files = [join(path, f) for f in listdir(path) if isfile(join(path, f)) and f[:-4].split('-')[-1] == 'n{}'.format(n)]
    instances = list()
    counter = 0
    for f in files:
        counter += 1
        with open(f, 'r') as data:
            data = data.readlines()
            alpha = round(float(data[1]) / float(data[3]), 1)
            (depot_x, depot_y, _) = data[7].split(' ')
            depot = Node(1, float(depot_x), float(depot_y))
            nodes = [depot]
            node_id = 1
            for i in data[9:]:
                node_id += 1
                (x, y, _) = i.split(' ')
                node = Node(node_id, float(x), float(y))
                nodes.append(node)
        instance = Instance(counter, nodes, alpha)
        instance.compute_travel_times()
        instances.append(instance)
    return instances

def parse_poikonen(n):
    #path = './data/instances_poikonen/Table4LocationsFull.csv'
    #data = pd.read_csv(path, header)
    #print()
    pass