import osmnx
from common import Node
from search import hill_climbing
import time
from numba import jit




reference = (43.661667, -79.395)
#G = osmnx.graph_from_point(reference, dist=500, clean_periphery=True, simplify=True)
G = osmnx.load_graphml('data.graphml')
origin = Node(graph=G, osmid=1907446268)
destination = Node(graph=G, osmid=1633421938)
s = time.process_time()
route = hill_climbing(G,origin,destination)
e = time.process_time()
print(e-s)
