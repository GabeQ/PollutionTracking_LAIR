#!/usr/bin/env python

'''Test script for checking graphAndGrid.py'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

from graphAndGrid import *
import networkx as nx

G = nx.Graph()
nodes = [0, 1, 2, 3, 4]
edges = [(0, 1), (0, 2), (1, 2), (1, 4), (2, 3), (3, 4)]
G.add_nodes_from(nodes)
G.add_edges_from(edges)
cartCoords = [(1, 1), (1, 3), (1.5, 1.5), (2, 2), (1, 1.5)]
dic = {}
for i in range(len(cartCoords)):
	dic.update({nodes[i]: cartCoords[i]})

nx.set_node_attributes(G, 'cartesian_coords', dic)
grid = make_grid_from_graph(G, 1)
