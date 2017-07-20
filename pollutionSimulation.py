#!/usr/bin/env python

'''pollutionSimulation.py: Contains functions for setting up pollution parameters within a graph'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import networkx as nx, numpy as np
from haversine import haversine


def get_pollution_sources(graph):
	'''Returns a list of nodes connected to motorways (aka freeways) which should be labeled as pollution sources'''
	pollutionSources = []
	for e in graph.edges():
		if 'motorway' in graph.edge[e[0]][e[1]][0]['highway'] or 'motorway_link' in graph.edge[e[0]][e[1]][0]['highway']:
			pollutionSources += [e[0], e[1]]
	return pollutionSources


def pollution_calc_sim(source, location, meters = False):
	'''Calculate a the amount of pollution at a certain point given a simulated
	pollution source. The amount of pollution increase exponentially the closer
	the location is to the source. Distance from the source is calculated using the
	haversine formula, which uses the great circle distance between two points. Source
	and location should be tuples of latitude and longitude pairs.'''

	distance = haversine(source, location)
	if meters == True:
		distance *= 100
	pollution = np.exp(-distance)
	return 100*pollution


def add_pollution_to_routeNodes(graph, pollutionSource, nodeList):
	'''Adds a pollution attribute each node in a given list of nodes and a coordinate
	for the pollution source (which should be given as a tuple).'''

	pollutionDic = {}
	for i in range(len(nodeList)):
		currentNode = nodeList[i]
		pollutionAmount = pollution_calc_sim(pollutionSource, (graph.node[currentNode]['y'], graph.node[currentNode]['x']))
		pollutionDic.update({currentNode : pollutionAmount})

	nx.set_node_attributes(graph, 'pollution_amount', pollutionDic)


def add_pollution_to_routeEdge(graph, pollutionSource, edgeList):
	'''Adds a pollution attribute each node in a given list of nodes and a coordinate
	for the pollution source (which should be given as a tuple). Takes the average of
	the pollution of two nodes to create a pollution attribute for the edge.'''

	pollutionDic = {}

	for i in range(len(edgeList)):
		node1 = edgeList[i][0]
		node2 = edgeList[i][1]
		pollutionAmount1 = pollution_calc_sim(pollutionSource, (graph.node[node1]['y'], graph.node[node1]['x']))
		pollutionAmount2 = pollution_calc_sim(pollutionSource, (graph.node[node2]['y'], graph.node[node2]['x']))
		edgePollution = (pollutionAmount1 + pollutionAmount2)/2
		pollutionDic.update({edgeList[i] + (0,): edgePollution})

	nx.set_edge_attributes(graph, 'pollution_amount', pollutionDic)


def add_pollution_all_nodes(graph, pollutionSourceList, osmID = False):
	'''Given multiple pollution sources, calculates the pollution at all nodes in a graph. pollutionSourceList
	is a list of latitude, longitude tuples. If pollutionSOurceList is a list of osmID nodes, set osmID = True'''
	pollutionDic = {}
	sources = pollutionSourceList
	if osmID:
		for i in range(len(sources)):
			sources[i] = (graph.node[pollutionSourceList[i]]['y'], graph.node[pollutionSourceList[i]]['x'])

	for currentNode in graph.nodes():
		pollutionAmount = 0

		for source in sources:
			pollutionAmount += pollution_calc_sim(source, (graph.node[currentNode]['y'], graph.node[currentNode]['x']))

		pollutionAmount /= len(pollutionSourceList)
		pollutionDic.update({currentNode : pollutionAmount})

	nx.set_node_attributes(graph, 'pollution_amount', pollutionDic)


def transfer_graph_node_pollution(graph1, graph2):
	'''transers data attributes from graph1 to a graph2'''
	commonNodes = list(set(graph1.nodes()).intersection(graph2.nodes()))
	pollutionDic = {}
	for n in commonNodes:
		pollutionDic.update({n: graph1.node[n]['pollution_amount']})

	nx.set_node_attributes(graph2, 'pollution_amount', pollutionDic)

