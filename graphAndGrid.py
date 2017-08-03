#!/usr/bin/env python

'''graphandgrid.py: Contains functions for analyzing both the 2D grid and the graph of road networks together.'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import networkx as nx, math as m
from streetNetworkGraphing import get_cart_coords
from grid import *
from collections import deque


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
				grid.update_pollutionEst_nearby_cells(pollution, xCoord, yCoord, updateDist)
			else:
				grid.update_pollutionEst_all_cells(pollution, xCoord, yCoord)

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


def find_best_grid_path(streetGrid, pollutionMap, startPoint, missionTime, velocity):
	'''Creates an initial path given the starting streetGrid and corresponding streetNetwork from the
	startPoint to the startPoint. The path should not take longer than the missionTime going at the
	specified velocity and the cells should not overlap. missionTime should be in seconds and velocity
	in meters/second. The algorithm navigates through the path based on the pollution map, which contains
	a pollution measurement and its corresponding x and y position.'''
	maxDistance = missionTime * velocity #max distance that can be traveled given the mission time and constant velocity
	maxCells = int(maxDistance / streetGrid.cellSize) #max cells that can be visited given the cell size and the max distance
	for value in pollutionMap:
		streetGrid.update_pollutionEst_all_cells(value[0], value[1], value[2])
	startCell = streetGrid.get_closest_cell(startPoint[0], startPoint[1])
	paths = list(nx.all_simple_paths(streetGrid.graph, startCell.center, startCell.center, maxCells))
	bestPath = None
	bestPathScore = 0
	for path in paths:
		simGrid = streetGrid
		pathScore = 0
		for point in path:
			curSimCell = simGrid.get_closest_cell(point[0], point[1])
			curSimCell.cell_objective_function(.899)
			pathScore += curSimCell.cost
		if pathScore > bestPathScore:
			bestPath = path
			bestPathScore = pathScore
	return bestPath
