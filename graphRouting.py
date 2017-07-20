#graphRouting.py
#Gabriel Quiroz 2017
from pollutionMapping_LAIR import nodeList_to_edgeList

def graphRouting(graph, route):
	'''Given a graph and a specific route of nodes, print directions to get from point A to point B'''
	path = nodeList_to_edgeList(route)

	for edge in path:
		node1 = edge[0]
		node2 = edge[1]
		name = graph.edge[node1][node2][0]['name']
		dy = graph.node[node2]['y'] - graph.node[node1]['y']
		dx = graph.node[node2]['x'] - graph.node[node1]['x']

		if dy == 0:
			ydir = ''
		elif dy > 0:
			ydir = 'NORTH'
		else:
			ydir = 'SOUTH'

		if dx == 0:
			xdir = ''
		elif dx > 0:
			xdir = 'EAST'
		else:
			xdir = 'WEST'

		print('Go' +' '+ ydir + ' ' + xdir + ' ' + 'on' + ' ' + name)
