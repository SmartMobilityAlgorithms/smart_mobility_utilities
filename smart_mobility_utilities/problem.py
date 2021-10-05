""" Provides some utilities for specific problems/algorithms like heuristic functions """

import math
import random
import itertools
import numpy as np

from .common import *


########################################################################
########################################################################
############################# A STAR ###################################
########################################################################
########################################################################

"""Calculates the straight line distance between two points on earth
(specified in decimal degrees). This is of course an approximation, but 
acceptable one if the two points are close to each other

Parameters
----------
lon1: longitude of the first point
lat1: latitude of the second point
lon2: longitude of the second point
lat2: latitude of the second point

Returns
-------
distance: the straight line distance calculated by pythagoras theorem

"""
def straight_line(lon1, lat1, lon2, lat2):
    return math.sqrt((lon2 - lon1)**2 + (lat2-lat1)**2)



"""Calculates the great circle distance between two points  on the earth
(specified in decimal degrees). This is useful when finding distance between 
two points that are far-away. 

Parameters
----------
lon1: longitude of the first point
lat1: latitude of the second point
lon2: longitude of the second point
lat2: latitude of the second point

Returns
-------
distance: the distance of the great circle arc between two points
          calculated by harversine method
"""

def haversine_distance(lon1, lat1, lon2, lat2):
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 

    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r

"""Used in A-star algorithm; it takes the source and destination node
and calculate the summation of straight line distance between each node
to origin and each node to the destination.

The value of the summation is not the actual distance but up to the some actual scale as 
we use matplotlib coordinates (x,y)

Parameters
----------
G: NetworkX graph returned from osmnx methods
origin: The id of the origin node in the graph 
destination: The id of the destination node in the graph
measuring_dist: The method used in measuring distance between nodes; staright line distance,
                harversine distance

Returns
-------
distanceGoal, distanceOrigin: dictionaries that associate the proper distance with node id

"""

def astar_heuristic(G, origin, destination, measuring_dist = straight_line):
    distanceGoal = dict()
    distanceOrigin = dict()

    originX = G.nodes[origin]['x']
    originY = G.nodes[origin]['y']

    destX = G.nodes[destination]['x']
    destY = G.nodes[destination]['y']

    for node in G:
        pointX = G.nodes[node]['x']
        pointY = G.nodes[node]['y']

        originDist = measuring_dist(originX, originY, pointX, pointY)
        destDist = measuring_dist(pointX, pointY, destX, destY)

        distanceGoal[node] = originDist
        distanceOrigin[node] = destDist

    return distanceGoal, distanceOrigin



##########################################################################################################
##########################################################################################################


########################################################################
########################################################################
############################# Simulated Annealing ######################
########################################################################
########################################################################



"""Schedule exponential function for cooling in simulated annealing

Parameters
----------
k: starting temperature of the function
lam: the rate of temperature decreasing
limit: number of iterations after which the function produces zero value

Returns
-------
function: it is lambda function that take the current number of iteration
"""

def exp_schedule(k=20, lam=0.005, limit=100):
    function = lambda t: (k * np.exp(-lam * t) if t < limit else 0)
    return function



##########################################################################################################
##########################################################################################################

########################################################################
########################################################################
############################# Genetic Algorithm ########################
########################################################################
########################################################################

#######################################################################
############################ For Shortest Path Problem ################
#######################################################################

"""Mutation policy for routes. It fails a random node in the route and
stitch the resulting gap and that is it. Sometimes this process fail, so
we iterate until it succeed.

Parameters
----------
G: NetworkX graph returned from osmnx methods
route: The route to be mutated
number_of_mutation: Number of mutation 

Returns
-------
path: mutated version from the route
"""
def mutate(G, route):
    source = route[0]
    destination = route[len(route) - 1]

    failed = random.choice(route)

    path = shortest_path_with_failed_nodes(G, route, source, destination, [failed])

    # This method could fail because of a lot of factors relating to the graph structure
    # Check the documentation fo the shortest_path_with_failed_nodes to lear more
    while path == math.inf:
        failed = random.choice(route)
        path = shortest_path_with_failed_nodes(G, route, source, destination, [failed])
    
    return path

"""Crossover between two routes. This is more or less 1-point crossover, but choosing the crossover
node isn't random because sometimes it is not possible to crossover two routes. We choose the crossover
node by searching both routes to find a node that is common between them and split at that node. This route
may not be there, in that case we just return the first route without doing anything.

Parameters
----------
route_1: The first route
route_2: The second route

Returns
-------
The result of doing crossover between route_1 and route_2
"""

def cross_over(route_1, route_2):
    origin = route_1[0]
    destination = route_1[len(route_1) - 1]

    intersection = [*itertools.filterfalse(\
                    lambda element : element in [origin, destination] ,\
                    list(set(route_1) & set(route_2)))]
    
    if len(intersection) == 0: return route_1 # if there is not common node, just return the first route

    cross_over_point = random.choice(intersection)
    first_point = route_1.index(cross_over_point)
    second_point = route_2.index(cross_over_point)

    if probability(0.5):
        return route_1[:first_point] + route_2[second_point:]
    else:
        return route_2[:second_point] + route_1[first_point:]
    
"""Probabilistically choose number of elements from a pool based
on their probability distribution/their value.

Parameters
----------
pool: pool of choices you have to choose from
num_of_choices: number of elements that would be choosed from the list
probability_dist: probability distribution that describes the relative weight 
                  of each element in the pool

Returns
-------
a list of the choices made
"""
def select_best(pool, num_of_choices, probability_dist):
        return random.choices(population=pool, weights=probability_dist, k= num_of_choices)


#########################################################################
############################# Combinatorial Problems ####################
#########################################################################

#
#                       Crossover Operators
#

"""This function does partially mapped crossover between two permutations.
It works on any list of elements that supports equality operator.

Parameters
----------
firstPermutation: The first parent to go into crossover
secondPermutation: The second parent to go into crossover

Returns
-------
child: The product of crossing-over both parents permutations
"""


def PMX_crossover(firstPermutation, secondPermutation):
    # we need to know the length of either permutation
    # they must be equal in size
    length = len(firstPermutation)
    
    # (1) choosing the two crossover points
    #     by randomly select a point from the
    #     first half and another point from the second half
    first_Cross = random.randint(0, length // 2)
    second_Cross = random.randint(length // 2 + 1, length - 1) + 1
    
    # (2) initializing the two equal sized segments
    #     and create another array with the same size of 
    #     any permutation to be the child
    child = [None] * length
    subP1 = firstPermutation[first_Cross:second_Cross]
    subP2 = secondPermutation[first_Cross:second_Cross]
    
    # (3) copy the elements in the segment from the first permutation
    #    into the same segment in the child
    child[first_Cross:second_Cross] = subP1
    
    # (4) finding common elements in the segment from the
    #     the first permutation and the second permutation
    #     and get its mirror from first permutation to second
    pairs = list()
    for element in subP2:
        if element not in subP1:
            pairs.append((element, subP1[subP2.index(element)]))
    
    # (5) copying into the child all the elements in the segment
    #.    that are present in the first permutation segment but
    #.    aren't present in the second permutation segment.
    #.    if not we need to copy that element in place outside
    #     the segment in a place where we are sure that would
    #.    result into inadmissible permutations.
    for pair in pairs:
        second = pair[1]
        if second not in subP2:
            index = secondPermutation.index(second)
            child[index] = pair[0]
        else:
            # when there is an element from the segment of the first
            # permutation in the segment of the second permutation 
            reflect = firstPermutation[secondPermutation.index(second)]
            
            # bouncing back and forth between the two arrays indices
            # to get out of second permutation segment
            while reflect in subP2:
                bounce = reflect
                reflect = firstPermutation[secondPermutation.index(bounce)]
            child[secondPermutation.index(reflect)] = pair[0]
    
    # (6) go through all the elements that have not been assigned
    #     yet in the child array and assign them with the second permutation
    #     elements
    for i in range(length):
        if child[i] == None:
            child[i] = secondPermutation[i]
    return child

"""This function does edge crossover between two permutations.
It only works on networkx graph permutations.

Parameters
----------
firstPermutation: The first parent to go into crossover
secondPermutation: The second parent to go into crossover
G: the networkx graph that the two permutations are generated from

Returns
-------
child: The product of crossing-over both parents permutations
"""

def ERO_crossover(G, firstPermutation, secondPermutation):
    # constructing edge table
    edgeTable = dict()
    elements = firstPermutation[:]
    length = len(elements)
    
    # just like adjacency list of nodes
    # in a given graph, but it is actually
    # the result of union between the two given
    # adjacency lists of a certain parent from both graphs
    for source in elements:
        edgeTable[source] = list()
        firstPermutationAdj = G[firstPermutation.index(source)]
        secondPermutationAdj = G[secondPermutation.index(source)]
        adjList = list(set().union(firstPermutationAdj, secondPermutationAdj))
        edgeTable[source]= adjList

    child = list()
    parent = random.choice(elements)
    elements.remove(parent)
    
    # terminate when the length of the child is the same
    # as the length of their parent
    while len(child) < length:
        child.append(parent)

        # remove the parent from all the adjacency lists
        for adjList in edgeTable.values():
            try:
                adjList.remove(parent)
            except:
                pass

        parentAdjList = edgeTable[parent][:]
        del edgeTable[parent]

        if len(parentAdjList) == 0: continue
        parent = min(parentAdjList, key = lambda parent : len(edgeTable[parent]))

    return child

"""This function does order one crossover between two permutations.
It works on any list of elements that supports equality operator.

Parameters
----------
firstPermutation: The first parent to go into crossover
secondPermutation: The second parent to go into crossover

Returns
-------
child: The product of crossing-over both parents permutations
"""

def ordOne_crossover(firstPermutation, secondPermutation):
    length = len(firstPermutation)
    
    # choose the start and the end of the segment 
    # to be copied from the first parent
    start_Segment = random.randint(0, length // 2)
    end_Segment = random.randint(length // 2 + 1, length - 1) + 1
    
    # create a child
    child = list()
    
    # add the randomaly selected segment from the first parent
    child.extend(firstPermutation[start_Segment: end_Segment])
    
    # add what is left from the second parent that wasn't added from the first parent
    residueFromSegment = list(set(secondPermutation) - set(firstPermutation[start_Segment: end_Segment]))
    child.extend(residueFromSegment)
    
    return child



#########################################################################
############################# Combinatorial Problems ####################
#########################################################################

#
#                       Mutation Operators
#

"""This function does insert mutation over the given permutation.
It chooses two random genes from the list and makes sure that they
are next to each other by shifting the list.

It preserves most of the order and adjacency information

Parameters
----------
permuation: The permutation to be mutated

Returns
-------
child: The mutated permutation
"""
def insert_mutation(permutation):
    # copying the list so we don't mess with the original
    child = permutation[:]
    
    # choose two random genes and make sure that they are different
    first_gene = random.choice(child)
    second_gene = random.choice(child)
    while first_gene == second_gene:
        first_gene = random.choice(child)
        second_gene = random.choice(child)
    
    # removing the second gene from the list and insert it just after the first
    child.remove(second_gene)
    geneNewIndex = child.index(first_gene) + 1
    child.insert(geneNewIndex, second_gene)
    return child

"""This function does swap mutation over the given permutation.
It chooses two random genes from the list and swap them together.

Parameters
----------
permuation: The permutation to be mutated

Returns
-------
child: The mutated permutation
"""
def swap_mutation(permutation):
    # copying the list so we don't mess with the original
    child = permutation[:]

    length = range(len(child))

    # choose two random gene position so they could be swaped
    first_gene_pos, second_gene_pos = random.sample(length, 2)

    # swapping 
    child[first_gene_pos], child[second_gene_pos] =\
    child[second_gene_pos], child[first_gene_pos] 

    return child