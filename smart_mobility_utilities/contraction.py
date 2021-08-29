import networkx
import math

def dijkstra_with_contraction(G, source, destination, contracted = None):
    networkx.set_node_attributes(G, {contracted: True}, 'contracted')
        
    shortest_path = dict()
    heap = list()
    
    for i in G.nodes():
        if not networkx.get_node_attributes(G, 'contracted')[i]:
            shortest_path[i] = math.inf
            heap.append(i)
    shortest_path[source] = 0
    
    while len(heap) > 0:
        
        q = min(heap, key = lambda node : shortest_path[node])
        if q == destination:
            networkx.set_node_attributes(G, {contracted: False}, 'contracted')
            return shortest_path[q]
        heap.remove(q)
        
        for v in G[q]:
            # if the node is contracted we skip it
            if not networkx.get_node_attributes(G, 'contracted')[v]:
                distance = shortest_path[q] + G[q][v]['weight']
                if distance < shortest_path[v]:
                    shortest_path[v] = distance
                    
    networkx.set_node_attributes(G, {contracted: False}, 'contracted')
    
    return math.inf # if we can't reach the destination

def calculate_edge_difference(G, shortest_paths):
    edge_difference = list()
    seenBefore = list()
    
    for i in G.nodes():
        # used in edge difference calculations
        edges_incident = len(G[i])

        # we will be deleting the node entry
        # from the original shortest paths
        # dictionary so we need to save its state
        # for later iterations
        contracted_node_paths = shortest_paths[i]
        del shortest_paths[i]
        
        # excluding the node that we have just contracted
        new_graph = [*G.nodes()] 
        new_graph.remove(i)
        
        # let's compute the new shortest paths between
        # the nodes of the graph without the contracted
        # node so we can see the changes and add arcs 
        # to the graph accordingly but that is in
        # the algorithm itself 
        new_shortest_paths = dict()

        for source in new_graph:
            new_shortest_paths[source] = dict()
            for destination in new_graph:
                # path the contracted node "i" to compute new shortest paths accordingly
                new_shortest_paths[source][destination] = dijkstra_with_contraction(G, \
                                                                                    source, \
                                                                                    destination, \
                                                                                    contracted = i)
        # the add arcs to keep the graph all pairs shortest paths invariant
        shortcuts = 0

        for source in new_shortest_paths:
            # we get a copy from the original and the new shortest paths dictionary
            SP_contracted = new_shortest_paths[source]
            SP_original = shortest_paths[source]
            for destination in SP_contracted:
                # this is statement so we don't add 2 arcs
                # for the same pair of nodes 
                if [source, destination] in seenBefore: continue
                seenBefore.append(sorted((source,destination)))
                
                # if there is a difference between the original SP and
                # post-contraction SP -- just add new arc
                if SP_contracted[destination] != SP_original[destination]:
                    shortcuts += 1
        
        # let's leave the dictionary as we took it 
        # from the last iteration
        shortest_paths[i] = contracted_node_paths
        
        # this is the value of the contraction
        # heuristic for that node
        ED = shortcuts - edges_incident
        edge_difference.append((i, ED))
    return edge_difference