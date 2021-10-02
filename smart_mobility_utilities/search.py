"""
A collection of graph search algorithms, as seen in the examples from the book.

@TODO: 
1. Code cleanup
2. Code commenting improvements

"""
import networkx
from collections import deque
from smart_mobility_utilities.common import *
from typing import List
from smart_mobility_utilities.children import get_children, get_beam
from smart_mobility_utilities.problem import astar_heuristic
from smart_mobility_utilities.contraction import *
import heapq
import math
from time import process_time


def BFS(origin:Node, destination: Node):
    route = []
    frontier = deque([origin])
    explored = set()
    found = False

    while frontier and not found:
        node = frontier.popleft()
        explored.add(node)
        for child in node.expand():
            if child not in explored and child not in frontier:
                if child == destination:
                    route = child.path()
                    found = True
                frontier.append(child)
    return route

def DFS(origin:Node, destination: Node):
    
    route = []
    frontier = deque([origin])
    explored = set()
    found = False

    while frontier and not found:
        node = frontier.pop()
        explored.add(node)
        for child in node.expand():
            if child not in explored and child not in frontier:
                if child == destination:
                    route  = child.path()
                    found = True
                    continue
                frontier.append(child)
    return route

def dijkstra(G: networkx.MultiDiGraph, origin:Node, destination:Node):

    # Using a set here avoids the problem with self loops
    seen = set()
    shortest_dist = {osmid: math.inf for osmid in G.nodes()}
    unrelaxed_nodes = [Node(graph = G, osmid = osmid) for osmid in G.nodes()]

    shortest_dist[origin.osmid] = 0
    found = False

    while len(unrelaxed_nodes) > 0 and not found:
    
        node = min(unrelaxed_nodes, key = lambda node : shortest_dist[node.osmid])
        
        # relaxing the node
        unrelaxed_nodes.remove(node)
        seen.add(node.osmid)
            
        # if the destination node has been relaxed then that is the route we want
        if node == destination:
            route = node.path()
            found = True
            continue
        
        # otherwise, let's relax edges of its neighbours
        for child in node.expand():
            # skip self-loops
            
            if child.osmid in seen: continue
            
            child_obj = next((node for node in unrelaxed_nodes if node.osmid == child.osmid), None)
            child_obj.distance = child.distance
            
            distance = shortest_dist[node.osmid] + child.distance
            if distance < shortest_dist[child_obj.osmid]:
                shortest_dist[child_obj.osmid] = distance
                child_obj.parent = node
            
    return route

def hill_climbing(G:networkx.MultiDiGraph, 
                origin:Node, 
                destination:Node, 
                num_children:int=20, 
                multiprocessing:bool=False,
                workers:int=4):
    
    current = randomized_search(G, origin.osmid, destination.osmid)
    neighbours= get_children(G,current,num_children,multiprocessing,workers)
    shortest = min(neighbours , key = lambda route : cost(G, route))

    while cost(G, shortest) < cost(G, current):
        current = shortest
        neighbours = get_children(G,current,num_children,multiprocessing,workers)
        shortest = min(neighbours , key = lambda route : cost(G, route))

    return current
    
def beam(G:networkx.MultiDiGraph, 
        origin:Node, 
        destination:Node, 
        num_neighbours:int=10, 
        k:int=10, 
        multiprocessing:bool=False,
        workers:int=4
        ):
    seen = set()
    beam = [randomized_search(G,origin.osmid,destination.osmid) for _ in range(k)]
   
    # the seen routes must be converted to a tuple to be hashable to be stored in a set
    for route in beam: seen.add(tuple(route))
    pool = []
    if multiprocessing:
        children = get_beam(G,beam,num_neighbours,multiprocessing=True,workers=workers)
    else:
        children = get_beam(G,beam,num_neighbours)
    for child in children:
        for node in child:
            if tuple(node) in seen: continue
            else: 
                pool.append(node)
                seen.add(tuple(node))
    pool += beam
    last_beam = None
    while beam != last_beam:
        last_beam = beam
        beam = heapq.nsmallest(k, pool, key = lambda route: cost(G, route))
        
        for route in beam: seen.add(tuple(route))    
        pool = []
        if multiprocessing:
            children = get_beam(G,beam,num_neighbours,multiprocessing=True,workers=workers)
        else:
            children = get_beam(G,beam,num_neighbours)
        for child in children:
            for node in child:
                if tuple(node) in seen: continue
                else: pool.append(node); seen.add(tuple(node))
        pool += beam   
    route = min(beam, key = lambda route : cost(G, route)) 
    return route

def astar(G: networkx.MultiDiGraph, origin: Node, destination: Node):
    # Get the A* Heuristic for all the nodes in the graph
    toOrigin, toDestination = astar_heuristic(G, origin.osmid, destination.osmid)
    route = []
    frontier = list()

    frontier.append(origin)
    explored = set()
    found = False

    while frontier and not found:    
        # choose a node based on its heuristic value
        node = min(frontier, key = lambda node : toOrigin[node.osmid] + toDestination[node.osmid])
        frontier.remove(node)
        explored.add(node)
        
        # expand its children
        for child in node.expand():
            if child not in explored and child not in frontier:
                if child == destination:
                    route = child.path()
                    found = True
                    continue
                frontier.append(child)

    return route

def bidirectional_astar(G: networkx.MultiDiGraph, origin: Node, destination: Node):

    # define destination and origin for the backwards expansion
    destination_b = origin
    origin_b = destination

    # get A*
    toOrigin_f, toDestination_f = astar_heuristic(G, origin.osmid, destination.osmid)
    toOrigin_b, toDestination_b = astar_heuristic(G, origin_b.osmid, destination_b.osmid)

    route = []

    f_value = lambda node: toOrigin_f[node.osmid] + toDestination_f[node.osmid]
    b_value = lambda node: toOrigin_b[node.osmid] + toDestination_b[node.osmid]

    frontier_f = list()
    frontier_b = list()

    frontier_f.append(origin)
    frontier_b.append(origin_b)

    explored_f = list()
    explored_b = list()

    collide = False
    found = False
    altr_expand = False # to alternate between front and back

    while frontier_f and frontier_b and not collide and not found:
    
        if altr_expand:
            # remove node_f from frontier_f to expand it
            node = min(frontier_f, key = lambda node : f_value(node))
            frontier_f.remove(node)
            explored_f.append(node)
            
            for child in node.expand():
                if child in explored_f: continue
                
                if child == destination:
                    route = child.path()
                    found = True
                    break

                # checking for collusion with the target expansion
                if child in explored_b:
                    overlapped = next((node for node in explored_b if node == child))
                    # we don't take the overlapped node twice
                    route = child.path()[:-1] + overlapped.path()[::-1]
                    collide = True
                    break

                frontier_f.append(child)
            altr_expand = False
        else:
            # remove node_b from frontier_b to expand it
            node = min(frontier_b, key = lambda node : b_value(node))
            frontier_b.remove(node)
            explored_b.append(node)
            
            for child in node.expand():
                if child in explored_b: continue
                if child == destination_b:
                    route = child.path()[::-1] # we reverse the list because we expand from the back
                    found = True
                    break

                if child in explored_f:
                    overlapped = next((node for node in explored_f if node == child), None)
                    route = overlapped.path()[:-1] + child.path()[::-1]
                    collide = True
                    break

                frontier_b.append(child)
            altr_expand = True
    return route

def contraction_hierarchy_generate(G):
    # Remove self-loops and parallel edges
    clean_G = networkx.DiGraph()
    for u in G.nodes:
        for v in G[u]:
            if u == v: G.remove_edge(u,v); continue
            if len(G[u][v]) > 1:
                keep = min(G[u][v], key=lambda x:G[u][v][x]['length'])
                keep = G[u][v][keep]
                clean_G.add_edge(u,v,**keep)
                continue
            clean_G.add_edge(u,v,**G[u][v][0])

    G = clean_G

    # Using functions from smart_mobility_utilities, we can generate shortest paths and edge differences very quickly
    
    sp = shortest_paths(G)
    ed = edge_differences(G,sp)
    edges_before = [*G.edges()]
    contract_graph(G,ed,sp) # Contract the graph according to ED
    hierarchical_order = dict()
    for order, node in enumerate(ed):
        hierarchical_order[node] = order
    
    return G, hierarchical_order

def bidirectional_dijkstra_with_contraction(G, start, end, hierarchical_order, cost = False):
    # Define the start and end
    start = 36603405
    end = 24959560

    # Generate the upward and downward graphs
    up, up_SP = generate_dijkstra(G,start,hierarchical_order)
    down, down_SP = generate_dijkstra(G,end,hierarchical_order,'down')

    minimum = math.inf
    merge_node = None

    for x in up_SP:
        if x == start or x == end: continue
        if up_SP[x] == math.inf or down_SP[x] == math.inf: continue
        total = up_SP[x] + down_SP[x]
        if total < minimum: 
            minimum = total
            merge_node = x

    route1 = build_route(G,start,merge_node,up)
    route2 = build_route(G,end,merge_node,down)
    if cost: return route1[:-1] + route2[::-1], minimum
    return route1[:-1] + route2[::-1]






def benchmark(algorithm,use_G=True,include_cost=True,**kwargs):
    G = kwargs.pop('G') if not use_G else kwargs['G']
    start_time = process_time()
    result = algorithm(**kwargs)
    end_time = process_time()
    result_cost = cost(G,result) if include_cost else 0
    return algorithm.__name__,end_time-start_time, result_cost

