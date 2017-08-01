#!/usr/bin/env python

'''graphandgrid.py: Contains functions for analyzing both the 2D grid and the graph of road networks together.'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import networkx as nx, math as m
from streetNetworkGraphing import get_cart_coords
from grid import *


def roundup(x, num):
	'''rounds a number up to the nearest number of base num'''
	return m.ceil(x/float(num))*num


def rounddown(x, num):
	'''rounds a number down to the nearest number of base num'''
	return m.floor(x/float(num))*num


def make_grid_from_graph(graph, roundNum = 10):
	'''Makes a grid with size specified from the graph's cartesian coordinates.'''
	cartCoords = get_cart_coords(graph)
	xCoords = [coord[0] for coord in cartCoords]
	yCoords = [coord[1] for coord in cartCoords]
	left = min(xCoords)
	bottom = min(yCoords)
	right = max(xCoords)
	top = max(yCoords)

	dx = right - left
	dy = top - bottom
	dx = roundup(dx, roundNum) + roundNum
	dy = roundup(dy, roundNum) + roundNum

	numCol = dx/roundNum
	numRow = dy/roundNum
	cellSize = roundNum
	origin = (rounddown(left, roundNum) - roundNum, rounddown(bottom, roundNum) - roundNum)
	return Grid2D(int(numCol), int(numRow), cellSize, gridOrigin = origin)


def pollutionTracking_sim(graph, grid, routeList, updateDist = None):
	'''Takes in a graph with pollution measurements, a corresponding grid,
	and a route to update the grid with pollution estimates using the pollution
	measurements on the graph nodes as actual pollution values.'''
	for route in routeList:
		for n in route:
			pollution = graph.node[n]['pollution_amount']
			xCoord = graph.node[n]['cartesian_coords'][0]
			yCoord = graph.node[n]['cartesian_coords'][1]

			if updateDist:
				grid.update_nearby_cells(pollution, xCoord, yCoord, updateDist)
			else:
				grid.update_all_cells(pollution, xCoord, yCoord)

	grid_pollution_surf_plotly(grid)


def get_cell_from_node(graph, grid, node):
	'''Performs the grid function of getting the closest cell from a x and y position
	using a specific node'''
	xCoord, yCoord = graph.node[node]['cartesian_coords']
	cell = grid.get_closest_cell(xCoord, yCoord)
	return cell


def get_nodes_in_cell(graph, grid, cell):
	'''Given a cell in a grid and the grid's corresponding graph, get all nodes whose
	cartesian coordinates are within the cell'''
	xMid, yMid = cell.center
	length = grid.cellSize
	left = xMid - (length/2)
	right = xMid + (length/2)
	bottom = yMid - (length/2)
	top = yMid + (length/2)
	nodeList = []

	for n in graph.nodes():
		x, y = graph.node[n]['cartesian_coords']
		if left <= x <= right:
			if bottom <= y <= top:
				nodeList.append(n)

	return nodeList

def get_nodes_multiple_cells(graph, grid, cell, cellDist):
	iCurrent, jCurrent = cell.get_cell_ID()
	xStart = iCurrent - cellDist
	yStart = jCurrent - cellDist
	xEnd = iCurrent + cellDist + 1
	yEnd = jCurrent + cellDist + 1
	nodeList = []

	if xStart < 0:
		xStart = 0
	if yStart < 0:
		yStart = 0

	if xEnd >= grid.numCol:
		xEnd = grid.numCol
	if yEnd >= grid.numRow:
		yEnd = grid.numRow

	for i in range(xStart, xEnd):
		for j in range(yStart, yEnd):
			cell = grid.get_cell(i, j)
			nodeList += get_nodes_in_cell(graph, grid, cell)

	return nodeList


def connect_neighbor_grids(grid1, grid2):
	'''Connects the cells between two grids that are next to each other and have
	equal resolutions. Returns a graph of the two connected grids'''
	graph = nx.compose(grid1.graph, grid2.graph)
	origin1 = grid1.origin
	origin2 = grid2.origin
	delta = (origin2[0] - origin1[0], origin2[1] - origin1[1])
	#Test to see if grids are actually next to each other
	if delta[0] > grid1.cellSize*grid1.numCol or delta[1] > grid1.cellSize*grid1.numRow:
		return "Grids are not next to each other"
	elif delta[0] > 0 and delta[1] > 0:
		return "Grids are diagonal to each other"
	#Find out where grids reside using delta and connect cells together
	elif delta[0] > 0:
		for row in range(grid1.numRow):
			cell1 = grid1.get_cell(grid1.numCol - 1, row)
			cell2 = grid2.get_cell(0, row)
			graph.add_edge(cell1.center, cell2.center)
	elif delta[0] < 0:
		for row in range(grid1.numRow):
			cell1 = grid1.get_cell(0, row)
			cell2 = grid2.get_cell(grid2.numCol - 1, row)
			graph.add_edge(cell1.center, cell2.center)
	elif delta[1] > 0:
		for col in range(grid1.numCol):
			cell1 = grid1.get_cell(col, grid1.numRow - 1)
			cell2 = grid2.get_cell(col, 0)
			graph.add_edge(cell1.center, cell2.center)
	else:
		for col in range(grid1.numCol):
			cell1 = grid1.get_cell(col, 0)
			cell2 = grid2.get_cell(col, grid2.numRow - 1)
			graph.add_edge(cell1.center, cell2.center)
	return graph


def increase_grid_resolution(graph, grid, curNode):
	cell = get_cell_from_node(graph, grid, curNode)
	print(cell)
	if type(cell) == Grid2D:
		increase_grid_resolution(graph, cell, curNode)
	else:
		nodeList = get_nodes_in_cell(graph, grid, cell)
		print(nodeList)
		if len(nodeList) > 1:
			xCoord, yCoord = graph.node[curNode]['cartesian_coords']
			grid.update_resolution(xCoord, yCoord, resolution = 5)
			print(grid.cells)
			increase_grid_resolution(graph, grid, curNode)
		else:
			return


# def resolution_routing(graph, grid, route, newPath):
# 	'''Given a graph, its corresponding grid, and a route from node A to node B,
# 	use updated resolution in the grid to replan the route'''
# 	if route == []:
# 		return newPath
# 	else:
# 		node1 = route.pop[0]
# 		node2 = route.pop[1]
# 		cell1 = get_cell_from_node(graph, grid, node1)
# 		cell2 = get_cell_from_node(graph, grid, node2)
# 		if cell1 == cell2:
# 			nodeList = get_nodes_in_cell(graph, grid, cell1)
# 			xCoord, yCoord = graph.node[node1]['cartesian_coords']
# 			while len(get_nodes_in_cell(graph, grid, cell1)) > 1:
# 				grid.update_resolution(xCoord, yCoord, resolution = 2)
