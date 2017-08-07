#!/usr/bin/env python

'''pollutionGraphing.py: Contains functions to graph pollution data, either with matplotlib or plotly'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

from streetNetworkGraphing import get_lat_long_coords, get_cart_coords
from grid import *
import osmnx as ox, matplotlib.cm as cm, plotly.plotly as py, plotly.graph_objs as go


def graph_with_node_pollution(G, pollutionAmountList):
	nc = []
	norm = cm.colors.Normalize(vmin=min(pollutionAmountList), vmax=max(pollutionAmountList))
	cmap = cm.ScalarMappable(norm=norm, cmap=cm.Reds)

	for n in G.nodes():
		if 'pollution_amount' in G.node[n]:
			nc.append(cmap.to_rgba(G.node[n]['pollution_amount']))
		else:
			nc.append((0, 0, 0, 1,))
	ox.plot_graph(G, node_color = nc, node_size=20, node_zorder = 3)


def graph_with_edge_pollution(graph):
	ec = []
	pollutionPerEdge = []
	edgePolList = []
	for e in graph.edges():
		if 'pollution_amount' in graph.edge[e[0]][e[1]][0]:
			pollutionPerEdge.append(graph.edge[e[0]][e[1]][0]['pollution_amount'])
			edgePolList.append(e)

	norm = cm.colors.Normalize(vmin = 0, vmax = max(pollutionPerEdge))
	cmap = cm.ScalarMappable(norm=norm, cmap=cm.GnBu)

	for e in graph.edges():
		if 'pollution_amount' in graph.edge[e[0]][e[1]][0]:
			ec.append(cmap.to_rgba(graph.edge[e[0]][e[1]][0]['pollution_amount'], alpha = 0))
		else:
			ec.append((1, 0, 0, 0,))

	ox.plot_graph(graph, edge_color = ec, edge_linewidth=2.5)
	return ec


def graph_pollution_mesh_plotly(graph):
	'''Takes a graph and plots a 3D mesh plot of pollution. x-values are longitude, y-values
	are latitudes, and z-values are pollution values. If a node has no pollution attribute,
	then pollution at that node is 0.'''

	cartCoords = get_cart_coords(graph)
	x = [cartCoords[i][0] for i in range(len(cartCoords))]
	y = [cartCoords[i][1] for i in range(len(cartCoords))]
	pollution = []

	for n in graph.nodes():
		if 'pollution_amount' in graph.node[n]:
			pollution.append(graph.node[n]['pollution_amount'])
		else:
			pollution.append(0.0)

	trace = go.Mesh3d(x=x,y=y,z=pollution,color='90EE90',opacity=0.50)
	py.plot([trace])


def graph_pollution_surf_plotly(graph):
	'''Takes a graph and plots the surface plot of pollution. X and Y values are both in meters away from the
	origin (southernmost node).'''

	latCoords = [coord[0] for coord in get_lat_long_coords(graph)]
	longCoords = [coord[1] for coord in get_lat_long_coords(graph)]
	nodes = graph.nodes()
	cartCoords = get_cart_coords(graph)
	x = [cartCoords[i][0] for i in range(len(cartCoords))]
	y = [cartCoords[i][1] for i in range(len(cartCoords))]
	pollution = []

	for i in range(len(x)):
		xNodeData = []

		for j in range(len(y)):
			if longCoords[i] == graph.node[nodes[i]]['x'] and latCoords[j] == graph.node[nodes[i]]['y']:
				if 'pollution_amount' in graph.node[nodes[i]]:
					xNodeData.append(graph.node[nodes[i]]['pollution_amount'])
			else:
				xNodeData.append(0.0)

		pollution.append(xNodeData)

	data = [go.Surface(x = x, y = y, z = pollution)]
	fig = go.Figure(data=data)
	py.plot(fig, filename='graph_pollution_data')


def grid_pollution_surf_plotly(grid):
	'''Takes a grid with pollution estimates in each cell and plots the estimates in a mesh plot'''
	col = grid.numCol
	row = grid.numRow
	x = []
	y = []
	pollution = []

	for xPos in range(col):
		x.append(grid.cells[xPos][0].center[0])

	for yPos in range(row):
		y.append(grid.cells[0][yPos].center[1])

	for j in range(row):
		polData = []

		for i in range(col):
			cell = grid.get_cell(i, j)
			polData.append(cell.polEst)

		pollution.append(polData)

	data = [go.Surface(x = x, y = y, z = pollution)]
	fig = go.Figure(data = data)
	py.plot(fig, filename = 'grid_pollution_data')
