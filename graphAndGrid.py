#!/usr/bin/env python

'''graphandgrid.py: Contains functions for analyzing both the 2D grid and the graph of road networks together.'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import networkx as nx, math as m
from streetNetworkGraphing import get_cart_coords
from grid import *


def roundup(x):
	'''rounds a number up to the nearest hundred'''
	return int(m.ceil(x/100.0))*100


def rounddown(x):
	'''rounds a number down to the nearest hundred'''
	return int(m.floor(x/100.0))*100


def make_grid_from_graph(graph):
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
	dx = roundup(dx) + 200
	dy = roundup(dy) + 200

	numCol = dx/100
	numRow = dy/100
	cellSize = 100
	origin = (rounddown(left) - 100, rounddown(bottom) - 100)
	return Grid2D(int(numCol), int(numRow), cellSize, origin)


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
		if left < x < right:
			if bottom < y < top:
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
			



