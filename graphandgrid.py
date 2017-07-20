#graphandgrid.py
#Gabriel Quiroz
#Contains functions for analyzing both the 2D grid and the graph of road networks

import networkx as nx, math as m
from pollutionMapping_LAIR import get_cart_coords
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







