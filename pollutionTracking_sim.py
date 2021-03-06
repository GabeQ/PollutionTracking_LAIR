#!/usr/bin/env python

'''Script that sets up variables for pollution tracking simulation'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"
import osmnx as ox, networkx as nx, matplotlib.cm as cm, pandas as pd, numpy as np, plotly.plotly as py, plotly.graph_objs as go
from haversine import haversine

from streetNetworkGraphing import *
from pollutionSimulation import *
from pollutionGraphing import *
from graphRouting import *
from kalmanFiltering import *
from graphAndGrid import *
from grid import *
import sys
sys.setrecursionlimit(5000)


# #Claremont Graph with route from mudd to super king
# Claremont = ox.graph_from_place('Claremont, CA', network_type = 'drive')
# set_cart_coords(Claremont)
# pollutionSources = get_pollution_sources(Claremont)
# pollutionSource = (34.086703, -117.721367)
# add_pollution_all_nodes(Claremont, pollutionSources, osmID = True)
# # mudd = ox.get_nearest_node(Claremont, (34.1061, -117.7105))
# # king = ox.get_nearest_node(Claremont, (34.080117, -117.721439))
# # route = nx.shortest_path(Claremont, mudd, king, weight = 'length')
# # Claremont_grid = make_grid_from_graph(Claremont, 100)
#
#
# '''Global Variables for small tests on 1km area'''
# center = (34.107089, -117.720289)
# testGraph = ox.graph_from_point(center, distance = 500, network_type = 'drive')
# set_cart_coords(testGraph)
# transfer_graph_node_pollution(Claremont, testGraph)
# testGrid = make_grid_from_graph(testGraph, 250)

#Actual Test
origin = (34.094318, -117.722022)
north, south, east, west = 34.107280, 34.094410, -117.701701, -117.720921
Claremont = ox.graph_from_bbox(north, south, east, west, network_type = 'drive')
set_cart_coords(Claremont, origin)
closestNode = ox.get_nearest_node(Claremont, origin)
streetGrid = make_grid_from_graph(Claremont, 500)
# polMap = create_pollution_map("polMap_driving_1.txt")
# path = cell_decomp_routing(streetGrid, polMap, Claremont.node[closestNode]['cartesian_coords'], 1000, 5, 3)
# route = path_to_street(Claremont, path)
# graphRouting = graphRouting(Claremont, route)
# lat = [coord[0] for coord in get_lat_long_coords_route(Claremont, route)]
# lon = [coord[1] for coord in get_lat_long_coords_route(Claremont, route)]
