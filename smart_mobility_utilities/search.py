"""
A collection of graph search algorithms, as seen in the examples from the book.

@TODO: 
1. Multiprocessed versions
2. Code cleanup
3. Code commenting improvements

"""
import networkx
from collections import deque
from smart_mobility_utilities.common import *
from typing import List
from children import get_children, get_beam
import heapq


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
    import math

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
                starter:List[int]=None,
                multiprocessing:bool=False,
                workers:int=4):

    if starter is not None:
        current = starter
    else:
        current = randomized_search(G, origin.osmid, destination.osmid)
    
    neighbours = get_children(G,current,num_children,multiprocessing,workers)
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
        starter:List[int]=None,
        multiprocessing:bool=False,
        workers:int=4
        ):
    seen = set()
    beam = [randomized_search(G,origin.osmid,destination.osmid) for _ in range(k)]
    if starter is not None:
        beam[0] = starter
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



