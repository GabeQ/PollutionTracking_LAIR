#!/usr/bin/env python

'''Test script for checking graphAndGrid.py'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

from graphAndGrid import *
from grid import *
import networkx as nx, random

testGrid = Grid2D(4, 4, 16) #64 meters = cellLength
testGrid.connect_cells()
startPos = (7, 10)
vel = 4 #m/s
missionTime = 30 #seconds

polMap = [] #list of tuples that contains measPol, xPos, and yPos

for i in range(60): #how many pol values we want to have
	measPol = random.randint(30, 70)
	xPos = random.randint(testGrid.origin[0], testGrid.cellSize * testGrid.numCol)
	yPos = random.randint(testGrid.origin[1], testGrid.cellSize * testGrid.numRow)
	polMap.append((measPol, xPos, yPos))
