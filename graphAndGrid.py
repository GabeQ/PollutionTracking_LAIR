#!/usr/bin/env python

'''graphandgrid.py: Contains functions for analyzing both the 2D grid and the graph of road networks together.'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import networkx as nx, math as m
from streetNetworkGraphing import get_cart_coords
from grid import *
from collections import deque
import random


def roundup(x, num):
	'''rounds a number up to the nearest number of base num'''
	return m.ceil(x/float(num))*num


def rounddown(x, num):
	'''rounds a number down to the nearest number of base num'''
	return m.floor(x/float(num))*num


def make_grid_from_graph(graph, roundNum = 50):
	'''Makes a grid with size specified from the graph's cartesian coordinates. Grid will
	have a cell size of roundNum, and have the number of columns and rows to account for the cell
	size.'''
	cartCoords = get_cart_coords(graph)
	xCoords = [coord[0] for coord in cartCoords]
	yCoords = [coord[1] for coord in cartCoords]
	left = min(xCoords)
	bottom = min(yCoords)
	right = max(xCoords)
	top = max(yCoords)
	dx = right - left
	dy = top - bottom
	dx = roundup(dx, roundNum)
	dy = roundup(dy, roundNum)
	numCol = int(dx/roundNum)
	numRow = int(dy/roundNum)
	cellSize = roundNum
	origin = (rounddown(left, roundNum), rounddown(bottom, roundNum))
	grid = Grid2D(numCol, numRow, cellSize, gridOrigin = origin)
	grid.connect_cells()
	return grid



def make_pollution_map(grid, numValues):
	'''Makes a pollution map which is a list of tuples. Each tuple contains
	measured pollution, xPosition, and yPosition in that order. all values are
	randomly generated'''
	polMap = []
	for i in range(numValues): #how many pol values we want to have
		measPol = random.randint(2000, 6000)
		xPos = random.randint(grid.origin[0], grid.cellSize * grid.numCol)
		yPos = random.randint(grid.origin[1], grid.cellSize * grid.numRow)
		polMap.append((measPol, xPos, yPos))
	return polMap


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


def make_grid_from_cells(currentGrid, cell1, cell2, startPoint, endPoint, resolution):
	'''Make a grid from two cells at a specific resolution per cell'''
	length = currentGrid.cellSize / 2
	center1 = cell1.center
	center2 = cell2.center
	delta = (center2[0] - center1[0], center2[1] - center1[1])
	if delta[0] == 0:
		if delta[1] > 0:
			xOrig, yOrig = center1[0] - length, center1[1] - length
			newCellSize = currentGrid.cellSize / resolution
			polEst = (cell1.polEst + cell2.polEst) / 2
			polEstVar = (cell1.polEstVar + cell2.polEstVar) / 2
			newGrid = Grid2D(resolution, resolution * 2, newCellSize, (xOrig, yOrig), parent = currentGrid, depth = currentGrid.depth + 1, polEst = polEst, polEstVar = polEstVar)
			newGrid.connect_cells()
			newGrid.start, newGrid.end = startPoint, endPoint
			currentGrid.children.append(newGrid)
			return newGrid
		else:
			xOrig, yOrig = center2[0] - length, center2[1] - length
			newCellSize = currentGrid.cellSize / resolution
			polEst = (cell1.polEst + cell2.polEst) / 2
			polEstVar = (cell1.polEstVar + cell2.polEstVar) / 2
			newGrid = Grid2D(resolution, resolution * 2, newCellSize, (xOrig, yOrig), parent = currentGrid, depth = currentGrid.depth + 1, polEst = polEst, polEstVar = polEstVar)
			newGrid.connect_cells()
			newGrid.start, newGrid.end = startPoint, endPoint
			currentGrid.children.append(newGrid)
			return newGrid
	else:
		if delta[0] > 0:
			xOrig, yOrig = center1[0] - length, center1[1] - length
			newCellSize = currentGrid.cellSize / resolution
			polEst = (cell1.polEst + cell2.polEst) / 2
			polEstVar = (cell1.polEstVar + cell2.polEstVar) / 2
			newGrid = Grid2D(resolution * 2, resolution, newCellSize, (xOrig, yOrig), parent = currentGrid, depth = currentGrid.depth + 1, polEst = polEst, polEstVar = polEstVar)
			newGrid.connect_cells()
			newGrid.start, newGrid.end = startPoint, endPoint
			currentGrid.children.append(newGrid)
			return newGrid
		else:
			xOrig, yOrig = center2[0] - length, center2[1] - length
			newCellSize = currentGrid.cellSize / resolution
			polEst = (cell1.polEst + cell2.polEst) / 2
			polEstVar = (cell1.polEstVar + cell2.polEstVar) / 2
			newGrid = Grid2D(resolution * 2, resolution, newCellSize, (xOrig, yOrig), parent = currentGrid, depth = currentGrid.depth + 1, polEst = polEst, polEstVar = polEstVar)
			newGrid.connect_cells()
			newGrid.start, newGrid.end = startPoint, endPoint
			currentGrid.children.append(newGrid)
			return newGrid


def find_best_grid_path(streetGrid, pollutionMap, missionTime, velocity, startPoint, endPoint = None, timeFactor = 3):
	'''Creates an initial path given the starting streetGrid and corresponding streetNetwork from the
	startPoint to the startPoint. The path should not take longer than the missionTime going at the
	specified velocity and the cells should not overlap. missionTime should be in seconds and velocity
	in meters/second. The algorithm navigates through the path based on the pollution map, which contains
	a pollution measurement and its corresponding x and y position.'''
	if endPoint == None:
		end = startPoint
	else: end = endPoint
	maxDistance = missionTime * velocity #max distance that can be traveled given the mission time and constant velocity
	maxCells = int(maxDistance / (streetGrid.cellSize * timeFactor)) #max cells that can be visited given the cell size and the max distance
	print('Number of cells that can be visited: ' + str(maxCells))
	for value in pollutionMap:
		streetGrid.update_pollutionEst_all_cells(value[0], value[1], value[2])
	startCell = streetGrid.get_closest_cell(startPoint[0], startPoint[1])
	endCell = streetGrid.get_closest_cell(end[0], end[1])
	minCells = len(nx.shortest_path(streetGrid.graph, startCell.center, endCell.center)) - 1
	if maxCells < minCells:
		print("Timeout for mission at current depth: " + str(streetGrid.depth))
		maxCells = minCells
	paths = list(nx.all_simple_paths(streetGrid.graph, startCell.center, endCell.center, maxCells))
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
	print(bestPath)
	return deque(bestPath)


def cell_decomp_routing(currentStreetGrid, pollutionMap, startPoint, initMissionTime, velocity, desiredDepth, newPath = []):
	#First case, for toplevel grid that has no path. Gives the toplevel grid a path based on the pollution map, adds mission time to toplevel grid
	print('Current route: ' + str(currentStreetGrid.route) + ', depth: ' + str(currentStreetGrid.depth))
	if currentStreetGrid.route == None and currentStreetGrid.depth == 0:
		print('Initial Mission Time: ' + str(initMissionTime))
		path = find_best_grid_path(currentStreetGrid, pollutionMap, initMissionTime, velocity, startPoint)
		path[0] = startPoint
		path[-1] = startPoint
		print('Initial Path:' + str(path))
		currentStreetGrid.route = path
		childMissionTime = int(currentStreetGrid.cellSize * (len(path) - 1) / velocity)
		print('Time for child: ' + str(childMissionTime))
		currentStreetGrid.add_child_grid_time(childMissionTime)
		return cell_decomp_routing(currentStreetGrid, pollutionMap, startPoint, None, velocity, desiredDepth, newPath = newPath)
	#Child case, for children grids that have no path. Gives the childgrid a path based on the pollution map, adds mission time to child grid
	elif currentStreetGrid.route == None and currentStreetGrid.depth != desiredDepth:
		path = find_best_grid_path(currentStreetGrid, pollutionMap, currentStreetGrid.parent.timeForChild,
			velocity, currentStreetGrid.start, endPoint = currentStreetGrid.end)
		print('Path from ' + str(currentStreetGrid.start) + ' to ' + str(currentStreetGrid.end) + ': ' + str(path))
		currentStreetGrid.route = path
		childMissionTime = int(currentStreetGrid.cellSize * (len(path) - 1) / velocity)
		print('Time: ' + str(childMissionTime))
		currentStreetGrid.add_child_grid_time(childMissionTime)
		return cell_decomp_routing(currentStreetGrid, pollutionMap, startPoint, None, velocity, desiredDepth, newPath = newPath)
	elif currentStreetGrid.route == None and currentStreetGrid.depth == desiredDepth:
		path = find_best_grid_path(currentStreetGrid, pollutionMap, currentStreetGrid.parent.timeForChild,
			velocity, currentStreetGrid.start, endPoint = currentStreetGrid.end)
		print('New Path from ' + str(currentStreetGrid.start) + ' to ' + str(currentStreetGrid.end) + ': ' + str(path))
		currentStreetGrid.route = path
		if newPath == []:
			newPath += list(path)
		else:
			newPath += list(path)[1:]
		parentGrid = currentStreetGrid.parent
		return cell_decomp_routing(parentGrid, pollutionMap, currentStreetGrid.end, None, velocity, desiredDepth, newPath = newPath)
	else:
		#Base case: If the current grid is the toplevel grid and the original route is empty, return the new path
		if len(currentStreetGrid.route) == 1 and currentStreetGrid.depth == 0:
			return newPath
		elif len(currentStreetGrid.route) == 1 and currentStreetGrid.depth != 0:
			parentGrid = currentStreetGrid.parent
			return cell_decomp_routing(parentGrid, pollutionMap, currentStreetGrid.end, None, velocity, desiredDepth, newPath)
		else:
			curPoint = currentStreetGrid.route.popleft()
			nextPoint = list(currentStreetGrid.route)[0]
			curCell = currentStreetGrid.get_closest_cell(curPoint[0], curPoint[1])
			nextCell = currentStreetGrid.get_closest_cell(nextPoint[0], nextPoint[1])
			childGrid = make_grid_from_cells(currentStreetGrid, curCell, nextCell, curPoint, nextPoint, 4)
			return cell_decomp_routing(childGrid, pollutionMap, curPoint, None, velocity, desiredDepth, newPath = newPath)


def get_node_from_point(graph, point):
	'''Gets the closest node on the graph from the specified point. Uses the graph's cartesian
	coordinates to get the closest point'''
	closestNode =  None
	smallestDist = m.inf
	for n in graph.nodes():
		xPos, yPos = graph.node[n]['cartesian_coords']
		dist = m.sqrt((xPos - point[0])**2 + (yPos - point[1])**2)
		if dist < smallestDist:
			closestNode = n
			smallestDist = dist
	return closestNode


def path_to_street(graph, gridPath):
	'''Takes a path created by cell_decomp_routing and maps it to the street network nodes of a graph'''
	streetNodePath = []
	for point in gridPath:
		node = get_node_from_point(graph, point)
		if streetNodePath == []:
			streetNodePath.append(node)
		else:
			prevNode = streetNodePath[-1]
			if node != prevNode:
				if node in graph.neighbors(prevNode):
					streetNodePath.append(node)
				else:
					path = nx.shortest_path(graph, prevNode, node)
					streetNodePath += path
	return streetNodePath
