#!/usr/bin/env python

'''pollutionMapping_LAIR.py: Contains basic functions for analyzing street network data from osmnx.
Cite to http://geoffboeing.com/2016/11/osmnx-python-street-networks/'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import osmnx as ox, networkx as nx, numpy as np
from collections import defaultdict, deque
earthRadius = 6.371e6


def get_lat_long_coords(graph):
	'''returns list of tuples giving latitude, longitude pair'''
	return [(graph.node[n]['y'], graph.node[n]['x']) for n in graph.nodes()]


def get_lat_long_coords_route(graph, route):
	'''returns list of latitude and longitude tuples for a given route'''
	return [(graph.node[n]['y'], graph.node[n]['x']) for n in route]


def set_cart_coords(graph, originCoord = None):
	'''Converts latitude longitude coordinates to cartesian coordinates (in meters) based off their distance from
	the origin. If the origin coordinate is none, chooses the southernmost node in the graph to be the origin'''
	latCoords = [coord[0] for coord in get_lat_long_coords(graph)]
	longCoords = [coord[1] for coord in get_lat_long_coords(graph)]

	if originCoord:
		originID = ox.get_nearest_node(graph, originCoord)
		origin = (graph.node[originID]['y'], graph.node[originID]['x'],)
	else:
		index = latCoords.index(min(latCoords))
		originID = graph.nodes()[index]
		origin = (graph.node[originID]['y'], graph.node[originID]['x'],)

	print("Origin set at:", origin)
	cartesianCoords = {}

	for n in graph.nodes():
		if n == originID:
			cartesianCoords.update({n : (0.0, 0.0)})
		else:
			xval = (graph.node[n]['x'] - origin[0])*(np.pi/180)*np.cos((graph.node[n]['y'] - origin[1])/2*np.pi/180)*earthRadius
			yval = (graph.node[n]['y'] - origin[1])*np.pi/180*earthRadius
			cartesianCoords.update({n: (xval, yval)})

	nx.set_node_attributes(graph, 'cartesian_coords', cartesianCoords)


def get_cart_coords(graph):
	return [graph.node[n]['cartesian_coords'] for n in graph.nodes()]


def nodeList_to_edgeList(nodeList):
	'''converts a list of neighbor nodes to their respective edges (used to convert
	a route of nodes to a route of edges).'''

	edgeList = []

	for i in range(len(nodeList) - 1):
		edgeList.append((nodeList[i], nodeList[i+1],))

	return edgeList
