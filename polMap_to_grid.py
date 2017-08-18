from surface_plot import *
from variance_plot import *
import csv
from grid import *

filename = 'Data\\170818\polMap_driving_2.txt'

with open(filename) as f:
    reader = csv.reader(f)
    dat = list(reader)

polMap = []
for point in dat:
    polEst = point[0][1:]
    polEstVar = point[1][1:]
    coord = (point[2][2:], point[3][1:-2])
    polMap.append([polEst, polEstVar, coord])

cellSize = 50
numRow = 30
numCol = 45
grid = Grid2D(numCol, numRow, cellSize, polEst = 4000, polEstVar = 1000)

for point in polMap:
    cell = grid.get_closest_cell(float(point[2][0]), float(point[2][1]))
    cell.polEst = float(point[0])
    cell.polEstVar = float(point[1])

surface_plot(grid)
variance_plot(grid)
