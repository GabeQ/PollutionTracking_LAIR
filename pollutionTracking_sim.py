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


#Claremont Graph with route from mudd to super king
Claremont = ox.graph_from_place('Claremont, CA', network_type = 'drive')
set_cart_coords(Claremont)
pollutionSources = get_pollution_sources(Claremont)
pollutionSource = (34.086703, -117.721367)
add_pollution_all_nodes(Claremont, pollutionSources, osmID = True)
mudd = ox.get_nearest_node(Claremont, (34.1061, -117.7105))
king = ox.get_nearest_node(Claremont, (34.080117, -117.721439))
route = nx.shortest_path(Claremont, mudd, king, weight = 'length')
Claremont_grid = make_grid_from_graph(Claremont, 100)


'''Global Variables for small tests on BBOX'''
center = (34.107089, -117.720289)
testGraph = ox.graph_from_point(center, distance = 1000, network_type = 'drive')
set_cart_coords(testGraph)
transfer_graph_node_pollution(Claremont, testGraph)
testGrid = make_grid_from_graph(testGraph, 100)
#Nodes to create path
start = ox.get_nearest_node(testGraph, (34.115018, -117.728304))
point2 = ox.get_nearest_node(testGraph, (34.098927, -117.729273))
point3 = ox.get_nearest_node(testGraph, (34.099659, -117.712609))
point4 = ox.get_nearest_node(testGraph, (34.107089, -117.720289))
#Paths selected and put into a pathlist
path1 = nx.shortest_path(testGraph, start, point2, weight = 'length')
path2 = nx.shortest_path(testGraph, point2, point3, weight = 'length')
path3 = nx.shortest_path(testGraph, point3, point4, weight = 'length')
path4 = nx.shortest_path(testGraph, point4, start, weight = 'length')
pathList = [path1, path2, path3, path4]
polList = [testGraph.node[n]['pollution_amount'] for n in testGraph.nodes()]
maxPolNode = testGraph.nodes()[polList.index(max(polList))]

