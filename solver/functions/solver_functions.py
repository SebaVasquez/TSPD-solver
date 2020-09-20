
from copy import copy
from networkx import strongly_connected_components, minimum_cut

def get_route(arcs, depot):
    route = [depot]
    arcs_temp = copy(arcs)
    aux = depot
    while arcs_temp:
        for idx, arc in enumerate(arcs_temp):
            if arc[0] == aux:
                route.append(arc[1])
                aux = arc[1]
                arcs_temp.pop(idx)
                break

    return tuple(route)

def get_route_recourse(x, arcs):
    recourse = {a: 0 for a in arcs}
    for a in x:
        recourse[a] = 1
    
    return recourse

def get_shortcuts(truck_route, max_drop):
    shortcuts = list()
    if max_drop > 0:
        shortcuts = [
            (truck_route[i], truck_route[j]) for i in range(len(truck_route) - 2) 
            for j in range(i + 2, len(truck_route)) if j <= i + 1 + max_drop
        ]
    
    return shortcuts

def get_disconnected_components(graph, depot, recourse):
    '''
        This function computes the strongly disconnected components of a given solution

        Args: DiGraph, depot, recourse

        Return: list of component(s)
    '''

    # The capacity of the arc is determined by the value of x
    x = recourse['x']

    graph.remove_edges_from(list(graph.edges))
    arcs = list()
    for (i, j) in x:
        if x[i, j] > 1e-8:
            arcs.append((i, j))
    graph.add_edges_from(arcs)
    components_list = list(strongly_connected_components(graph))
    selected_components = list()
    for component in components_list:
        if depot not in component and len(component) >= 2:
            selected_components.append(component)

    if selected_components: selected_components = [max(selected_components, key=lambda x: len(x))]

    return selected_components

def get_disconnected_component_mincut(graph, nodes, recourse):
    '''
        This function computes disconnected components of a given solution, solving a MinCut problem

        Args: DiGraph, list of nodes, recourse

        Return: list of component(s)
    '''

    capacity = recourse['x']
    gamma = recourse['gamma']
    depot = nodes[0]

    # Para todo j != depot, se hace min-cut. Sobre todos, se elige el subtour mas violado, junto al peso respectivo (gamma)
    graph.remove_edges_from(list(graph.edges))
    graph.add_edges_from(capacity)
    for (i, j) in capacity:
        graph[i][j]['capacity'] = capacity[i, j]
    components_list = list()
    component = None
    for i in nodes:
        if i != depot:
            (cap, (_, S)) = minimum_cut(graph, depot, i)
            if cap - gamma[j] < -1e-8:
                components_list.append((cap - gamma[i], i, S))
    
    if components_list: component = min(components_list, key=lambda x: x[0])

    return component     

