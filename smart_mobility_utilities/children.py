from multiprocessing import Pool
import math
from copy import deepcopy
from itertools import islice
import copy
from smart_mobility_utilities.common import Node

def shortest_path_with_failed_nodes_single(G,route,failed:list):
    i = 1
    j = len(route) -2
    result = shortest_path_with_failed_nodes(G,route,i,j,failed)
    if result is math.inf: return result
    path, i,j, r = result
    return r

def shortest_path_with_failed_nodes(G, route ,i,j, failed : list):
    source = route[i-1]
    target = route[j+1]
    origin = Node(graph = G, osmid = source)
    destination = Node(graph = G, osmid = target)

    ## you can't introduce failure in the source and target
    # node, because your problem will lose its meaning
    if source in failed: failed.remove(source)
    if target in failed: failed.remove(target)
    
    # if after removing source/target node from failed
    # list - just return math.inf which is equivalent to failure in search
    if len(failed) == 0: return math.inf

    # we need to flag every node whether it is failed or not
    failure_nodes = {node: False for node in G.nodes()}
    failure_nodes.update({node: True for node in failed})

    # we need to make sure that while expansion we don't expand
    # any node from the original graph to avoid loops in our route
    tabu_list = route[:route.index(source)] \
                + \
                route[route.index(target) + 1:] 

    # the normal implementation of dijkstra
    shortest_dist = {node: math.inf for node in G.nodes()}
    unrelaxed_nodes = [Node(graph = G, osmid = node) for node in G.nodes()]
    seen = set()

    shortest_dist[source] = 0

    while len(unrelaxed_nodes) > 0:
        node = min(unrelaxed_nodes, key = lambda node : shortest_dist[node])

        # if we have relaxed articulation nodes in our graph
        # halt the process -- we have more than one component
        # in our graph which makes the question of shortest path
        # invalid

        if shortest_dist[node.osmid] == math.inf: return math.inf

        if node == destination:
            return node.path(),i,j, route

        unrelaxed_nodes.remove(node); seen.add(node.osmid) # relaxing the node

        for child in node.expand():
            # if it is failed node, skip it
            if failure_nodes[child.osmid] or\
                child.osmid in seen or\
                child.osmid in tabu_list:
                continue

            child_obj = next((node for node in unrelaxed_nodes if node.osmid == child.osmid), None)
            child_obj.distance = child.distance

            distance = shortest_dist[node.osmid] + child.distance
            if distance < shortest_dist[child_obj.osmid]:
                shortest_dist[child_obj.osmid] = distance
                child_obj.parent = node

    # in case the node can't be reached from the origin
    # this return happens when the node is not on the graph
    # at all, if it was on a different component the second
    # return will be executed -- this is the third return
    
    return math.inf

def shortest_path_handler(args):
    return shortest_path_with_failed_nodes(*args)

def children_route(G, route, limit):
    results = []
    for i in range(1, len(route) - 1):
        for j in range(i, len(route) -1):
            # we can't work on the route list directly
            # because lists are passed by reference
            stitched = copy.deepcopy(route)
            failing_nodes = copy.deepcopy(route[i:j+1])
            args = shortest_path_with_failed_nodes(G, stitched, i,j, failing_nodes)
            if args == math.inf: continue
            to_be_stitched, k, l, r = args
            stitched[i:j+1] = to_be_stitched[1:-1]      # we need to skip the first and starting nodes of this route
                                                        # because these nodes already exit
            results.append(stitched)
            if len(results) == limit: return results

def children_route_handler(args):
    return children_route(*args)

def get_beam(G,routes, num_children=10, multiprocessing=False, workers=4):
    if multiprocessing:
        worker = ChildRoutesGenerator(workers=workers, G=G, route=routes[0], limit=num_children)
        results = worker.do_beam(routes)
        return results
    results = []
    for route in routes:
        children = children_route(G, route, num_children)
        results.append(children)
    return results

def get_children(G, route, num_children=20, multiprocessing=False, workers=4):
    if multiprocessing:
        worker = ChildRoutesGenerator(workers=workers,G = G, route=route, limit=num_children)
        worker.do_job()
        worker.pool.terminate()
        worker.pool.join()
        return worker.result
    return children_route(G,route,num_children)

class ChildRoutesGenerator():
    def __init__(self, workers, G, route, limit=20):
        self.workers = workers
        self.pool = Pool(processes=self.workers)
        self.result = []
        self.limit = limit
        self.G = G
        self.route = route

    def do_beam(self,routes):
        args = []
        results = []
        for r in routes:
            args.append((self.G,r,self.limit))
        for _ in self.pool.imap_unordered(children_route_handler,args, chunksize=1):
            results.append(_)
        self.pool.terminate()
        self.pool.join()
        return results

    def do_job(self, route=None, refresh=False):
        
        if refresh: self.pool = Pool(processes=self.workers)
        if route is not None:
            self.route = route
        args = []
        
        for i in range(1, len(self.route) - 1):
            for j in range(i, len(self.route) -1):
                stitched = deepcopy(self.route)
                failing_nodes = deepcopy(self.route[i:j+1])
                args.append((self.G, stitched, i, j, failing_nodes))
        
        for _ in self.pool.imap_unordered(shortest_path_handler,args, chunksize=1):
            if _ == math.inf: 
                continue
            new,i,j,old = _
            old[i:j+1] = new[1:-1]
            self.result.append(old)
            if len(self.result) == self.limit:
                break
        self.pool.terminate()
        self.pool.join()
        return self.result