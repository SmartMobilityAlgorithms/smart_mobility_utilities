""" Provides some utilities to ease the usage of ipyleaflet with osmnx """

import pandas, numpy
import ipyleaflet as lf
import folium as fl
import osmnx as ox
import networkx as nx

# Custom function to get center of graph, as osmnx.stats.extended_stats is now deprecated
def get_center(G):
    undir = G.to_undirected()
    length_func = nx.single_source_dijkstra_path_length
    sp = {source: dict(length_func(undir, source, weight="length")) for source in G.nodes}
    eccentricity = nx.eccentricity(undir,sp=sp)
    center_osmid = nx.center(undir,e=eccentricity)[0]
    return center_osmid

"""Draw leaflet map based for the graph passed to the function
with highlighting certain nodes with a marker. ipyleaflet rendering 
can be a little slow when dealing with graphs with many node, in that 
case when number of nodes > 1000, the function reverts to folium for rendering
which is a plain black and white rendering of the map not like ipyleaflet

Parameters
----------
G: networkx graph returned by osmnx
highlight: list of nodes to be marked on the leaflet map
zoom: the initial zooming level for the rendered map

Returns
-------
m: ipyleaflet/folium map for the osmnx graph with optional highlighted nodes
"""

def draw_map(G, highlight = None , zoom = 15, force_leaflet=False):
    """ draws ipyleaflet map with location as center of the map """
    center_osmid = get_center(G)
    #center_osmid = ox.stats.extended_stats(G,ecc=True)['center'][0]
    if len(G) >= 1000 and not force_leaflet:
        print(f"The graph has {len(G)} elements, using folium to improve performance.")
        if highlight:
            G_gdfs = ox.graph_to_gdfs(G)
            nodes_frame = G_gdfs[0]
            ways_frame = G_gdfs[1]
            m = ox.plot_graph_folium(G = G)
            for node_osmid in highlight:
                node = nodes_frame.loc[node_osmid]
                node_xy = [node['y'], node['x']]
                fl.Marker(node_xy).add_to(m)
        else: 
            m = ox.plot_graph_folium(G = G)
        return m
    G_gdfs = ox.graph_to_gdfs(G)
    nodes_frame = G_gdfs[0]
    ways_frame = G_gdfs[1]
    center_node = nodes_frame.loc[center_osmid]
    location = (center_node['y'], center_node['x'])
    m = lf.Map(center = location, zoom = zoom)

    for _, row in ways_frame.iterrows():
        lines = lf.Polyline(
            locations = [list(elem)[::-1] for elem in [*row['geometry'].coords]],
            color = "black",
            fill = False,
            weight = 1
        )
        m.add_layer(lines)

    # if we want to mark some specific nodes
    if highlight:
        for node_osmid in highlight:
            node = nodes_frame.loc[node_osmid]
            node_xy = (node['y'], node['x'])
            marker = lf.Marker(location = node_xy, draggable = False)
            m.add_layer(marker)

    return m

"""Draw leaflet an AntPath route based on the graph and the route passed 
to the function with highlighting the source and destination node of the route. 
ipyleaflet rendering can be a little slow when dealing with graphs with many node, in that 
case when number of nodes > 1000, the function reverts to folium for rendering
which is a plain black and white rendering of the map not like ipyleaflet

Parameters
----------
G: networkx graph returned by osmnx
route: list of nodes from the graph that represent connected route
zoom: the initial zooming level for the rendered map

Returns
-------
m: ipyleaflet/folium map for the osmnx graph with the route highlighted as AntPath layer
"""


def draw_route(G, route, zoom = 15, force_leaflet=False):
    
    center_osmid = get_center(G)
    G_gdfs = ox.graph_to_gdfs(G)
    nodes_frame = G_gdfs[0]
    ways_frame = G_gdfs[1]
    center_node = nodes_frame.loc[center_osmid]
    location = (center_node['y'], center_node['x'])
    

    start_node = nodes_frame.loc[route[0]]
    end_node = nodes_frame.loc[route[len(route)-1]]

    start_xy = (start_node['y'], start_node['x'])
    end_xy = (end_node['y'], end_node['x'])
    

    if len(route) >= 500 and not force_leaflet:
        print(f"The route has {len(G)} elements, using folium to improve performance.")
        m = ox.plot_route_folium(G = G, route = route, zoom= zoom, color='red')
        fl.Marker(location=start_xy).add_to(m)
        fl.Marker(location=end_xy).add_to(m)
        return m

    m = lf.Map(center = location, zoom = zoom)
    marker = lf.Marker(location = start_xy, draggable = False)
    m.add_layer(marker)
    marker = lf.Marker(location = end_xy, draggable = False)
    m.add_layer(marker)

    for u, v in zip(route[0:], route[1:]):
        try:
            geo = (ways_frame.query(f'u == {u} and v == {v}').to_dict('list')['geometry'])
            m_geo = min(geo,key=lambda x:x.length)
        except:
            geo = (ways_frame.query(f'u == {v} and v == {u}').to_dict('list')['geometry'])
            m_geo = min(geo,key=lambda x:x.length)
        x, y = m_geo.coords.xy
        points = map(list, [*zip([*y],[*x])])
        ant_path = lf.AntPath(
            locations = [*points], 
            dash_array=[1, 10],
            delay=1000,
            color='red',
            pulse_color='black'
        )
        m.add_layer(ant_path)

    return m
    