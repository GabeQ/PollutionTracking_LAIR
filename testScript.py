#!/usr/bin/env python

'''Test script for checking graphAndGrid.py'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

from graphAndGrid import *
import networkx as nx

G = nx.Graph()
nodes = [0, 1, 2, 3, 4, 5]
edges = [(0, 1), (0, 2), (1, 2), (1, 4), (1, 5), (2, 3), (3, 4), (4, 5) ]
G.add_nodes_from(nodes)
G.add_edges_from(edges)
cartCoords = [(1, 1), (2, 5), (7, 4), (1, 8), (4, 5), (8, 8)]
polEst = [32, 55, 22, 69, 87, 2]
cartDic = {}
polDic = {}
for i in range(len(cartCoords)):
	cartDic.update({nodes[i]: cartCoords[i]})
	polDic.update({nodes[i]: polEst[i]})

nx.set_node_attributes(G, 'cartesian_coords', cartDic)
nx.set_node_attributes(G, 'pollution_amount', polDic)
grid = Grid2D(3, 3, 3)
grid.connect_cells()
