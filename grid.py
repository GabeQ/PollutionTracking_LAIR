#!/usr/bin/env python

'''grid.py: Contains the classes of grid and cells, as well as methods to analyze pollution data within a cell.'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

from kalmanFiltering import meas_var_dist, kalman_filter
import numpy as np, networkx as nx


class Cell:

	def __init__(self, col, row, midpoint, polEst, polEstVar):
		self.col = col
		self.row = row
		self.center = midpoint
		self.polEst = polEst
		self.polEstVar = polEstVar
		self.cost = 0


	def __repr__(self):
		return 'Cell(xCenter = %s, yCenter = %s)' % (self.center[0], self.center[1])


	def get_cell_ID(self):
		'''Returns the column number and row number the column resides in
		from its parent grid'''
		return self.col, self.row


	def get_parent(self):
		'''Returns the parent grid that the cell belongs to.'''
		return self.parent


	def update_cell_state(self, measVal, xPos, yPos):
		'''Updates the parameters for the Kalman Filter'''
		distSquared = (xPos - self.center[0])**2 + (yPos - self.center[1])**2
		dist = np.sqrt(distSquared)
		measVar = meas_var_dist(dist) + .01
		posteriEst, posteriEstVar = kalman_filter(self.polEst, self.polEstVar, measVal, measVar)
		self.polEst = posteriEst
		self.polEstVar = posteriEstVar


	def cell_objective_function(self, alpha):
		'''Cost function of cell for optimizing the path taken in planning a route'''
		cost = alpha * self.polEstVar + (1 - alpha) * self.polEst
		self.cost = cost


class Grid2D:

	def __init__(self, numCol, numRow, cellSize, gridOrigin = (0, 0), parent = None, depth = 0, polEst = 100, polEstVar = 20):
		'''Initializes a 2D Grid given the length (x) and width (y) of the grid. The data parameter should be given as a 2D list.
		The length of the main list should be size X and the lenght of each list within the list should be size Y. The grid
		origin is the bottom left most point of the grid.'''
		self.numCol = numCol
		self.numRow = numRow
		self.cellSize = cellSize
		self.origin = gridOrigin
		self.parent = parent
		self.depth = depth
		self.children = []
		self.graph = nx.Graph()
		self.route = None
		self.start = None
		self.end = None
		self.timeForChild = None
		colDist = gridOrigin[0]
		colCells = []

		for col in range(numCol):
			rowDist = gridOrigin[1]
			rowCells = []

			for row in range(numRow):
				midpoint = (colDist + cellSize/2.0, rowDist + cellSize/2.0)
				cell = Cell(col, row, midpoint, polEst, polEstVar)
				rowCells.append(cell)
				self.graph.add_node(cell.center)
				rowDist += cellSize

			colCells.append(rowCells)
			colDist += cellSize

		self.cells = colCells


	def __repr__(self):
		return 'Grid(xOrigin = %s, yOrigin = %s)' % (self.origin[0], self.origin[1])


	def set_cell(self, col, row, cell):
		'''Sets a cell at the desired index of a grid'''
		self.cells[col][row] = cell


	def get_cell(self, col, row):
		'''Gets a cell given a specific row and column'''
		return self.cells[col][row]


	def get_closest_cell(self, xCoord, yCoord):
		'''Given x-coordinates and y-coordinates, return the cell that the coordinate lies
		in. If the coordinate lies between cells, always returns the upper right cell. If the
		coordinate lies between the uppermost edge or the rightmost edge of the grid, returns
		the next cell either to the bottom or to the left of the coordinate.'''
		xOrigin, yOrigin = self.origin
		xDiv = (xCoord - xOrigin)/self.cellSize
		yDiv = (yCoord - yOrigin)/self.cellSize
		xID = int(xDiv)
		yID = int(yDiv)
		if xID == self.numCol:
			xID -= 1
		if yID == self.numRow:
			yID -= 1
		return self.get_cell(xID, yID)


	def add_child_grid_time(self, time):
		self.timeForChild = time


	def connect_cells(self):
		'''Connects the nodes of the graph within the grid with their north, south, east, and west
		neighbors'''
		for col in range(self.numCol):
			for row in range(self.numRow - 1):
				curPoint = self.get_cell(col, row).center
				northPoint = self.get_cell(col, row + 1).center
				self.graph.add_edge(curPoint, northPoint)

		for row in range(self.numRow):
			for col in range(self.numCol - 1):
				curPoint = self.get_cell(col, row).center
				eastPoint = self.get_cell(col + 1, row).center
				self.graph.add_edge(curPoint, eastPoint)


	def update_pollutionEst_all_cells(self, measPolVal, xPos, yPos):
		'''Updates the polution estimate for all cells in the grid given the current position
		and a measured pollution value'''
		for i in range(self.numCol):
			for j in range(self.numRow):
				cell = self.get_cell(i, j)
				if type(cell) == Grid2D: #if the cell is actually another grid
					cell.update_pollutionEst_all_cells(measPolVal, xPos, yPos)
				else:
					cell.update_cell_state(measPolVal, xPos, yPos)


	def update_pollutionEst_nearby_cells(self, measPolVal, xPos, yPos, cellDistance = 2):
		'''Updates the polution estimate for cells within the cell distance of the current location'''
		closestCell = self.get_closest_cell(xPos, yPos)
		if type(closestCell) == Grid2D:
			closestCell.update_pollutionEst_nearby_cells(measPolVal, xPos, yPos, cellDistance)
		else:
			iCurrent, jCurrent = closestCell.get_cell_ID()
			xStart = iCurrent - cellDistance
			yStart = jCurrent - cellDistance
			xEnd = iCurrent + cellDistance + 1
			yEnd = jCurrent + cellDistance + 1

			if xStart < 0:
				xStart = 0
			if yStart < 0:
				yStart = 0

			if xEnd > self.numCol:
				xEnd = self.numCol
			if yEnd > self.numRow:
				yEnd = self.numRow

			for i in range(xStart, xEnd):
				for j in range(yStart, yEnd):
					cell = self.get_cell(i, j)
					if type(cell) == Grid2D: #If the cell is actually another grid
						cell.update_pollutionEst_all_cells(measPolVal, xPos, yPos)
					else:
						cell.update_cell_state(measPolVal, xPos, yPos)


	def cells_with_polEst_greaterThan(self, greaterThan, cellList = []):
		'''Returns a list of cells with pollution estimates greater than the selected value'''
		for col in range(self.numCol):
			for row in range(self.numRow):
				cell = self.get_cell(col, row)
				if type(cell) == Grid2D: #If cell is actually another grid
					cell.cells_with_polEst_greaterThan(greaterThan, cellList)
				else:
					if cell.polEst > greaterThan:
						cellList.append(cell)
		return cellList


	def cell_to_grid(self, cell, resolution):
		'''Converts a cell into a grid for higher resolution to analyze data'''
		xMid, yMid = cell.center
		length = self.cellSize
		xOrig = xMid - (length/2)
		yOrig = yMid - (length/2)
		newCellSize = length/resolution
		polEst = cell.polEst
		polEstVar = cell.polEstVar
		newGrid = Grid2D(resolution, resolution, newCellSize, (xOrig, yOrig), parent = self, depth = self.depth + 1, polEst = polEst, polEstVar = polEstVar)
		newGrid.connect_cells()
		self.graph.remove_node(cell.center)
		self.set_cell(cell.col, cell.row, newGrid)
