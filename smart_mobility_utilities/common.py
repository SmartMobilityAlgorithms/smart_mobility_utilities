"""
common stuff for gluing things together and hiding unnecessary complexity 
""" 

import random
import itertools
from collections import deque


"""
This is a wrapper around osmnx nodes so we can query a single node
with questions like that without having to deal with networkx dictionaries:
    * how did we get here from the origin?
    * what are my children?
    * what is my unique id
"""

class Node:
    # using __slots__ for optimization
    __slots__ = ['node', 'distance', 'parent', 'osmid', 'G']
    # constructor for each node
    def __init__(self ,graph , osmid, distance = 0, parent = None):
        # the dictionary of each node as in networkx graph --- still needed for internal usage
        self.node = graph[osmid]
        
        # the distance from the parent node --- edge length
        self.distance = distance
        
        # the parent node
        self.parent = parent
        
        # unique identifier for each node so we don't use the dictionary returned from osmnx
        self.osmid = osmid
        
        # the graph
        self.G = graph
    
    # returning all the nodes adjacent to the node
    def expand(self):
        children = [Node(graph = self.G, osmid = child, distance = self.node[child][0]['length'], parent = self) \
                        for child in self.node]
        return children
    
    # returns the path from that node to the origin as a list and the length of that path
    def path(self):
        node = self
        path = []
        while node:
            path.append(node.osmid)
            node = node.parent
        return path[::-1]
    
    # the following two methods are for dictating how comparison works

    def __eq__(self, other):
        try:
            return self.osmid == other.osmid
        except:
            return self.osmid == other
            
    
    def __hash__(self):
        return hash(self.osmid)

"""
Given an iterable with nodes ids and the networkx graph
The function calculated the weight of the route
"""

def cost(G, route):
    weight = 0
    for u, v in zip(route, route[1:]):
        weight += G[u][v][0]['length']   
    return round(weight,4)


"""
Given an itertable with nodes id and the networkx graph
The function will calculate the weight of the route as
if it is as tour, so after arriving at the last node
we will add the weight of the edge connecting the last node
with the first one
the expected route here is tuple
It was made to deal with simple graphs
"""
def cost_tour(G, route):
    weight = 0
    route = list(route)
    for u, v in zip(route, route[1:]+[route[0]]):
        weight += G[u][v]['weight']
    return weight



"""
Generate random simple path between source and destination node
over a osmnx graph.
We can use networkx.all_simple_paths iterator function which would serve the
same purpose, but if you went to the its implementation you will see that they use stack
of all the edges of the graph which would REALLY hurt our performance when you use big graph
over a complete city or something like that.
Our method is very simple randomized graph search, and the randomization is about randomly selecting
the node to expand in each step, and only keeping the frontier in our memory. It is obviously has time
complexity O(n+m) and space complexity O(n).
It is an iterator function so we don't overload our memory if you wanted a lot of paths.
"""
def randomized_search(G, source, destination):
    origin = Node(graph = G, osmid = source)
    destination = Node(graph = G, osmid = destination)
    
    route = [] # the route to be yielded
    frontier = deque([origin])
    explored = set()
    while frontier:
        node = random.choice(frontier)   # here is the randomization part
        frontier.remove(node)
        explored.add(node.osmid)

        for child in node.expand():
            if child not in explored and child not in frontier:
                if child == destination:
                    route = child.path()
                    return route
                frontier.append(child)

    raise Exception("destination and source are not on same component")


"""
Fix routes that are the product of merging two routes in opposite directions
by deleting the node that causes that and replace that gap with the shortest
path between the node before and after the gap.

This method was made to handle the routes generated from bi-directional
"""
def one_way_route(G, route):
    def isvalid(G, route):
        for u, v in zip(route, route[1:]):
            try:
                G[u][v]
            except:
                return False
        return True

    while True:
        if isvalid(G, route): break
        i = 0
        j = 1
        found = False
        while not found and j < len(route) - 1:          
            try:
                u, v = route[i], route[j]
                G[u][v]
                i+=1
                j+=1
            except:
                node_before = route[i]
                node_failing = route[i:j+1]
                node_after = route[j+1]
                output = shortest_path_with_failed_nodes(G, route,\
                                                                node_before,\
                                                                node_after,\
                                                                node_failing)
                while type(output) is not list and i > 1 and j < len(route) - 1:
                    i-=1
                    j+=1
                    node_before = route[i]
                    node_failing = route[i:j+1]
                    node_after = route[j+1]
                    output = shortest_path_with_failed_nodes(G, route,\
                                                                node_before,\
                                                                node_after,\
                                                                node_failing)
                route[i:j+2] = output
                found = True
                i+=1
                j+=1
    return route


"""
Yield random r-permutations of the list of nodes
"""
def random_tour(iterable, r=None, number_of_perms = 50):
    for i in range(number_of_perms):
        pool = tuple(iterable)
        r = len(pool) if r is None else r
        yield list(random.sample(pool, r))

"""
Return true with probability p.
"""
def probability(p):
    return p > random.uniform(0.0, 1.0)


"""
Flatten a list of lists
"""
def flatten(list2d):
    return list(itertools.chain(*list2d))


