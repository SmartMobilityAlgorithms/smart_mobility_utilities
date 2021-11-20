import requests
from ipywidgets import HTML
from ipyleaflet import Map, Marker, AntPath
import folium
import folium.plugins
import polyline


"""Class for creating POI (point of interest) with its full detailed geographic data
"""

class poi:
    """Initialize the POI by specifying the name of the place
    and the country in which the place exist and its OpenStreetMaps types

    We are relying on Nominatim API to query the OSM data. The name of
    the place could be just the name of the place like University of Toronto.

    You need to specify the name of the country so we can limit the search space,
    and as you probably imagine the same name of one place points to different places
    in different countries. You can find "London" in UK and in USA.

    As we have discussed in GettingStarted repositories there are three types of entities
    in OSM data, which is nodes and ways and relations. Nodes are just a place like a certain
    hospital or shop and ways/relations are just a set of nodes that specify some poly[gon-line]
    in a map. When we query Nominatim API with a certain address, it replies back with nodes/ways/relations
    that match that address, so we need the parameter osm_type to retrieve the right type.

    Parameters
    ----------
    name: the name/address of the POI
    country: the name of the country of the POI
    osm_type: data types of OSM entities [node - way - relation]

    Examples
    -------
    >>> UofT = poi("university of toronto", "canada")
    >>> UofT.coordinates
    ... (-79.3973638, 43.6620257)
    >>> UofT.address
    ... 'University of Toronto, St George Street, University—Rosedale, Old Toronto, Toronto, Peel, Golden Horseshoe, Ontario, M5T 2Z9, Canada'

    """
    def __init__(self, name, country, lat = 0, lng = 0):
            self.__geo_decode(name, country, lat, lng)

    """ Calls Nominatim API for geodecoding the address
    """

    def __geo_decode(self, name, country, lat=0, lng=0):

        # check https://nominatim.org/release-docs/develop/

        # try to issue this request from your terminal or smth to see the full response
        if lat != 0 and lng !=0:
            response = requests.get(f'https://nominatim.openstreetmap.org/reverse?lat={lat}&lon={lng}&format=geocodejson')
        else:
            response = requests.get(f'https://nominatim.openstreetmap.org/search?q={name} - {country}&format=geocodejson')

        if response.status_code != 200:
            raise ValueError("We couldn't decode the address, please make sure you entered it correctly")

        response_json = response.json()

        # the response may contain multiple places with
        # the same name, we will only take the first result
        # of the response which "probably" would be the place you wanted
        # that is why we get the index zero from the response
        # One other thing, maybe you wanted place "x" but there is
        # a way called "x" and a node called "x" (remember osm data types)
        # what you get could be the way not the node which you probably node
        # it won't matter at all most of the time but you need to know that
        index = 0
        place = response_json['features'][index] 

        self.address = place['properties']['geocoding']['label']
        self.osmid = place['properties']['geocoding']['osm_id']
        self.coordinates = tuple(place['geometry']['coordinates']) # (longitude, latitude)

    """Takes another poi object and find the route between the calling object and 
    the other object with specified mode of transportation like car - bike - foot.
    Driving mode is equivalent to car mode. If there are not many available mode 
    for the specified route, the API returns the car/driving mode by default.

    Parameters
    ----------
    destination: another poi object that would be the target of the route
    mode: this is the mode of transportation, there are three available modes: car/driving - bike - foot

    Returns
    -------
    Route dictionary: dictionary that consists of three keys
                    1. 'route' which is the (lat, log) coordinates that defines the route
                    2. 'length' the length of the route by meters
                    3. 'duration' this is how many seconds would it take to finish that route
                    OSRM calculates that based on multiple things like max speed
                    of the sub-routes and it also takes on consideration multiple things like
                    the number of turns in a given route and obstacles like gates in the roads
                    and such things. For EXACTLY how they do it please go to Project-OSRM/osrm-backend/profiles/car.lua
                    in their github account
    """
    def route_to(self, destination, mode = "driving", polyline=False):
        src = self.coordinates
        dest = destination.coordinates

        # check http://project-osrm.org/docs/v5.22.0/api/#general-options

        response = requests.get(f'http://router.project-osrm.org/route/v1/{mode}/{src[0]},{src[1]};{dest[0]},{dest[1]}?steps=true')
        response_json = response.json()

        if response_json['code'] != 'Ok':
            raise ValueError(f"OSRM couldn't find a route between {src} and {dest}")

        route = response_json["routes"][0]
        if polyline: return route
        cost = route["distance"]
        duration = route["duration"]
        legs = route["legs"][0]
        steps = legs["steps"]

        route_coords = list()
        for route_step in steps:
            maneuver = route_step["maneuver"]
            location = maneuver["location"]
            location = location[::-1]   # from (longitude, latitude) to (latitude, longitude) so ipyleaflet can handle it
            route_coords.append(tuple(location))
        
        return {'coords' : route_coords, 
                'length' : cost,
                'duration' : duration}
    
    def __eq__(self, other):
        return self.osmid == other.osmid

    def __hash__(self):
        return self.osmid

    # representing a POI object with the first sentence in the full address
    # and its OSM id -- you can easily use OSM id to retrieve POI objects from container
    def __repr__(self):
        name = self.address.split(",")[0]
        return f"Name: {name} ID: {self.osmid}"


##########################################################################################################
##########################################################################################################
##########################################################################################################
##########################################################################################################


"""This functions takes coordinates of a route between two POIs
and renders the route on ipyleaflet map.

It is meant to be used with the routes returned from poi objects.

Parameters
----------
route: is a list of coordinates point (lat, log)
zoom: is how much zoom the map would make on the route rendered

Returns
-------
m: ipyleaflet map
"""

def drawRoute(route, zoom = 12):
    # getting the center of the route
    m = Map(center = route[len(route) // 2], zoom = zoom)

    # mark the source node coordinates
    src_marker = Marker(location = route[0], draggable = False)
    m.add_layer(src_marker)

    # mark the destination node coordinates
    dest_marker = Marker(location = route[len(route) - 1], draggable = False)
    m.add_layer(dest_marker)

    # draw AntPath between every two consecutive nodes
    for u, v in zip(route[0:], route[1:]):
        step = map(list, [u, v])
        step_path = AntPath(
            locations = [*step],
            dash_array=[1, 10],
            delay=1000,
            color='black',
            pulse_color='red'
        )
        m.add_layer(step_path)

    return m

"""Renders POIs on ipyleaflet maps with popup containing the POI name

Parameters
----------
POIS: list of poi objects

Returns
-------
m: ipyleaflet map
"""

def drawPOIS(POIS, zoom=12):
    centerLat, centerLog = 0, 0

    # taking the average of all latitude and longitude of all the POIs
    # to get the center of the map 

    for poi in POIS:
        centerLat += poi.coordinates[0]
        centerLog += poi.coordinates[1]
    centerLat /= len(POIS)
    centerLog /= len(POIS)
    center = (centerLog, centerLat)

    m = Map(center=center, zoom=zoom, close_popup_on_click=False)

    # creating the popup messages on the markers
    # with the name of the POI
    for poi in POIS:
        name = poi.address.split(",")[0]
        marker = Marker(location=poi.coordinates[::-1])
        text = HTML()
        text.value = f"{name}"
        marker.popup = text
        m.add_layer(marker)

    return m

def number_DivIcon(color,number, prefix=""):
    """ Create a 'numbered' icon
    
    """
    icon = folium.DivIcon(
            icon_size=(150,36),
            icon_anchor=(12,40),
#             html='<div style="font-size: 18pt; align:center, color : black">' + '{:02d}'.format(num+1) + '</div>',
            html="""<span class="fa-stack " style="font-size: 12pt" >
                    <span class="fa fa-circle-o fa-stack-2x" style="color : {:s}"></span>
                    <strong class="fa-stack-1x">
                         {:s}{:d}  
                    </strong>
                </span>""".format(color,prefix,number)
        )

    return icon

def getRouteBounds(route):
    minLat = min(route, key=lambda x:x[0])
    maxLat = max(route, key=lambda x:x[0])
    minLng = min(route, key=lambda x:x[1])
    maxLng = max(route, key=lambda x:x[1])
    return [(minLat, minLng), (maxLat, maxLng)]

# This function draws POIS on a folium map, with markers designating route order.
def drawRouteOrder(route, POIS, order, zoom=12, colors=None, route_color='red', m=None, prefix=""):
    # POIS: list of coords
    if not m:
        m = folium.Map(zoom_start=zoom)
        bounds = getRouteBounds(route)
        m.fit_bounds(bounds)
    ordered_route = [POIS[x-1] for x in order]

    # Add markers
    for i in range(len(POIS)):
        loc = POIS[i].coordinates[::-1]
        folium.Marker(location=loc, icon=folium.Icon(color='white', icon_color='white')).add_to(m)
        if colors:
            color = colors[i]
        else:
            color='blue'

        folium.Marker(location=loc, icon=number_DivIcon(color,order.index(i+1)+1, prefix=prefix)).add_to(m)

    
    # Add path
    master_route = []
    for i in range(len(ordered_route)-1):
        n = ordered_route[i]
        n2 = ordered_route[i+1]
        r = n.route_to(n2, polyline=True)
        p = polyline.decode(r['geometry'])
        master_route.extend(p)
        
    
    folium.plugins.AntPath(
        locations = master_route,
        dash_array=[1, 10],
        delay=1000,
        color=route_color,
        pulse_color='orange'
    ).add_to(m)
        
    
    return m