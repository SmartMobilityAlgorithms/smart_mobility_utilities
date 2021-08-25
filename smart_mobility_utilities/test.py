import osmnx
from common import Node, cost, randomized_search
import time
import networkx
from typing import List
from children import get_children
import heapq
from tqdm import tqdm

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
    for route in beam:
        children = get_children(G,route,num_neighbours,multiprocessing,workers)
        for child in children:
            if tuple(child) in seen: continue
            else: 
                pool.append(child)
                seen.add(tuple(child))
    pool += beam
    last_beam = None
    print("Done initializing first beam")
    while beam != last_beam:
        last_beam = beam
        beam = heapq.nsmallest(k, pool, key = lambda route: cost(G, route))
        
        for route in beam: seen.add(tuple(route))    
        pool = []
        for route in beam:
            children = get_children(G,route,num_neighbours,multiprocessing,workers=2)
            for child in children:
                if tuple(child) in seen: continue
                else: pool.append(child); seen.add(tuple(child))
        pool += beam    
    route = min(beam, key = lambda route : cost(G, route)) 
    return route


reference = (43.661667, -79.395)
#G = osmnx.graph_from_point(reference, dist=500, clean_periphery=True, simplify=True)
G = osmnx.load_graphml('data.graphml')
origin = Node(graph=G, osmid=1907446268)
destination = Node(graph=G, osmid=1633421938)
s = time.process_time()
route = beam(G,origin,destination,multiprocessing=True,workers=2)
e = time.process_time()
print(f"Multiprocessed: \n    Time: {e-s}\n    Cost: {cost(G,route)}")
"""s2 = time.process_time()
route2 = beam(G,origin,destination)
e2 = time.process_time()
print(f"Multiprocessed: \n    Time: {e2-s2}\n    Cost: {cost(G,route2)}")"""
