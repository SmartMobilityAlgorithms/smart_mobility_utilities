import networkx as nx
import math
from copy import deepcopy

def shortest_paths(G):
    size = 0
    shortest_paths = {x:y for x,y in nx.shortest_path_length(G, weight='length')}
    for x in shortest_paths:
        shortest_paths[x] = {u:round(v,3) for u,v in shortest_paths[x].items()}
        size += len(shortest_paths[x])
    for u in G.nodes:
        for v in G.nodes:
            if v not in shortest_paths[u]:
                shortest_paths[u][v] = math.inf
                size +=1
    return shortest_paths

def edge_differences(G,sp):
    edge_diffs = dict()
    seen_list = []

    for node in G.nodes:
        edges_incident = len(G[node])
        if edges_incident == 1: # Handle terminating points
            edge_diffs[node] = -1
            continue
        new_graph = deepcopy(G)
        new_graph.remove_node(node)
        shortcuts = 0
        for neighbour in G[node]:
            for other_neighbour in G[node]:

                if neighbour == other_neighbour: continue
                if [neighbour,other_neighbour] in seen_list: continue
                seen_list.append([neighbour,other_neighbour])
                old_sp = sp[neighbour][other_neighbour]
                old_sp_rev = sp[other_neighbour][neighbour]
                try:
                    new_sp = nx.shortest_path_length(new_graph,neighbour,other_neighbour, weight='length')
                except:
                    new_sp = math.inf
                try:
                    new_sp_rev = nx.shortest_path_length(new_graph,other_neighbour,neighbour, weight='length')
                except:
                    new_sp_rev = math.inf
                need_new = old_sp != new_sp
                need_new_rev = old_sp_rev != new_sp_rev
                if need_new: shortcuts +=1
                if need_new_rev: shortcuts+=1
        ED = shortcuts - edges_incident
        edge_diffs[node] = ED
    return sorted(edge_diffs, key=lambda x:edge_diffs[x])

def contract_graph(G: nx.DiGraph, edge_difference, sp):
    # to keep track of the edges added after the algorithm finishes

    edge_diffs = dict()
    seen_list = []

    for node in edge_difference:
        edges_incident = len(G[node])
        if edges_incident == 1: continue # Terminating points

        new_graph = deepcopy(G)
        new_graph.remove_node(node)

        for neighbour in G[node]:
            for other_neighbour in G[node]:

                if neighbour == other_neighbour: continue
                if [neighbour,other_neighbour] in seen_list: continue
                seen_list.append([neighbour,other_neighbour])
                old_sp = sp[neighbour][other_neighbour]
                old_sp_rev = sp[other_neighbour][neighbour]
                try:
                    new_sp = nx.shortest_path_length(new_graph,neighbour,other_neighbour, weight='length')
                except:
                    new_sp = math.inf
                try:
                    new_sp_rev = nx.shortest_path_length(new_graph,other_neighbour,neighbour, weight='length')
                except:
                    new_sp_rev = math.inf
                need_new = old_sp != new_sp
                need_new_rev = old_sp_rev != new_sp_rev

                if need_new:
                    G.add_edge(neighbour,other_neighbour,length=old_sp,midpoint=node)
                if need_new_rev:
                    G.add_edge(other_neighbour,neighbour,length=old_sp_rev,midpoint=node)

def generate_dijkstra(G, source, hierarchical_order, direction = 'up', weight='length'):

    # initializing 
    SP = dict()
    parent = dict()
    unrelaxed = list()
    for node in G.nodes():
        SP[node] = math.inf
        parent[node] = None
        unrelaxed.append(node)
    SP[source] = 0

    # dijkstra
    while unrelaxed:
        node = min(unrelaxed, key = lambda node : SP[node])
        unrelaxed.remove(node)
        if SP[node] == math.inf: break
        for child in G[node]:
            # skip unqualified edges
            if direction == 'up':
                if hierarchical_order[child] < hierarchical_order[node]: continue
            if direction == 'down':
                if hierarchical_order[child] > hierarchical_order[node]: continue

            # If we're building a down graph, we need to use reverse weights
            if direction == 'down':
                if node not in G[child]: continue
                distance = SP[node] + G[child][node][weight]
            else:
                distance = SP[node] + G[node][child][weight]
            if distance < SP[child]:
                SP[child] = distance
                parent[child] = node
    return parent, SP

def merge_graphs(up_SP, down_SP):
    minimum = math.inf
    merge_node = None
    for i in up_SP:
        if down_SP[i] == math.inf: continue
        if down_SP[i] + up_SP[i] < minimum:
            minimum = down_SP[i] + up_SP[i]
            merge_node = i

    return minimum, merge_node

def build_route(G,origin, destination, parent):
    if destination not in G[origin]:
        # We need the parent of the destination instead
        return build_route(G,origin,parent[destination], parent) + [destination]
    
    edge = G[origin][destination]
    if 'midpoint' in edge and 'osmid' not in edge: # This is a contracted edge
        before = build_route(G,origin,edge['midpoint'], parent)
        after = build_route(G,edge['midpoint'], destination, parent)
        return before[:-1] + after
    return [origin,destination]



