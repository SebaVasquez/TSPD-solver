
import matplotlib
import matplotlib.pyplot as plt
import networkx as nx

def plot(instance, truck_arcs=list(), drone_arcs=list()):
    nodes = [i for i in instance.nodes]
    labels = {i: i for i in nodes if i != instance.depot_2}
    pos = {i: (i.x, i.y) for i in nodes}

    G = nx.DiGraph()
    nx.draw_networkx_nodes(G, pos, nodes, node_color='k', node_size=170, alpha=1)
    nx.draw_networkx_nodes(G, pos, nodes, node_color='w', node_size=140, alpha=1)

    nx.draw_networkx_edges(G, pos, edgelist=truck_arcs, width=1.5, arrows=True)
    nx.drawing.nx_pylab.draw_networkx_edges(G, pos, edgelist=drone_arcs, edge_color='r', style='dashed', width=0.75, arrows=True)

    nx.draw_networkx_labels(G, pos, labels, font_size=7)
    plt.axis('off')
    plt.show()

