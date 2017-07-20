#grid.py
#Gabriel Quiroz

from kalmanFiltering import meas_var_dist, kalman_filter
import numpy as np

class Cell:

	def __init__(self, col, row, midpoint):
		self.col = col
		self.row = row
		self.center = midpoint
		self.polEst = 100
		self.polEstVar = 20
		self.j = 0

	def get_cell_ID(self):
		return self.col, self.row

	def update_cell_state(self, measVal, xPos, yPos):
		'''Updates the parameters for the Kalman Filter'''
		distSquared = (xPos - self.center[0])**2 + (yPos - self.center[1])**2
		dist = np.sqrt(distSquared)
		measVar = meas_var_dist(dist)
		posteriEst, posteriEstVar = kalman_filter(self.polEst, self.polEstVar, measVal, measVar)
		self.polEst = posteriEst
		self.polEstVar = posteriEstVar

	def cell_cost_function(self, alpha):
		'''Cost function of cell for optimizing the path taken in planning a route'''
		cost = alpha * self.polEstVar + (1 - alpha) * self.polEst
		self.j = cost


class Grid2D:

	def __init__(self, numCol, numRow, cellSize, gridOrigin = (0, 0)):
		'''Initializes a 2D Grid given the length (x) and width (y) of the
		grid. The data parameter should be given as a 2D list. The length of
		the main list should be size X and the lenght of each list within the list
		should be size Y. The grid origin is the bottom left most point of the grid.'''
		self.numCol = numCol
		self.numRow = numRow
		self.cellSize = cellSize
		self.origin = gridOrigin
		colDist = gridOrigin[0]
		colCells = []

		for col in range(numCol):
			rowDist = gridOrigin[1]
			rowCells = []

			for row in range(numRow):
				midpoint = (colDist + cellSize/2.0, rowDist + cellSize/2.0)
				cell = Cell(col, row, midpoint)
				rowCells.append(cell)
				rowDist += cellSize

			colCells.append(rowCells)
			colDist += cellSize

		self.cells = colCells


	def set_cell(self, row, col, cell):
		'''Sets a cell at the desired index of a grid'''
		self.cells[col][row] = cell

	def get_cell(self, col, row):
		'''Gets a cell given a specific row and column'''
		return self.cells[col][row]

	def get_nearest_cell(self, xCoord, yCoord):
		'''Given x-coordinates and y-coordinates, return the cell that the coordinate lies
		in. If the coordinate lies in the center of 4 cells, returns the upper right cell'''
		xOrigin, yOrigin = self.origin
		xDiv = (xCoord - xOrigin)/self.cellSize
		yDiv = (yCoord - yOrigin)/self.cellSize
		xID = int(xDiv)
		yID = int(yDiv)
		return self.get_cell(xID, yID)

	def update_all_cells(self, measPolVal, xPos, yPos):
		'''updates the polution estimate for all cells in the grid given the current position
		and a measured pollution value'''
		for i in range(self.numCol):
			for j in range(self.numRow):
				cell = self.get_cell(i, j)
				cell.update_cell_state(measPolVal, xPos, yPos)

	def update_nearby_cells(self, measPolVal, xPos, yPos, cellDistance = 2):
		'''Updates the polution estimate for cells within the cell distance of the current location'''
		closestCell = self.get_nearest_cell(xPos, yPos)
		iCurrent, jCurrent = closestCell.get_cell_ID()
		xStart = iCurrent - cellDistance
		yStart = jCurrent - cellDistance
		xEnd = iCurrent + cellDistance
		yEnd = jCurrent + cellDistance

		if xStart < 0:
			xStart = 0
		if yStart < 0:
			yStart = 0

		if xEnd >= self.numCol:
			xEnd = self.numCol
		if yEnd >= self.numRow:
			yEnd = self.numRow

		for i in range(xStart, xEnd):
			for j in range(yStart, yEnd):
				cell = self.get_cell(i, j)
				cell.update_cell_state(measPolVal, xPos, yPos)

	def get_cell_with_polEst(self, greaterThan):
		cellList = []
		for col in range(self.numCol):
			for row in range(self.numRow):
				cell = self.get_cell(col, row)
				if cell.polEst > greaterThan:
					cellList.append(cell)
		return cellList




