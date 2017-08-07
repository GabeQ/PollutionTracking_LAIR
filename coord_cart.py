import numpy as np
import math as m
earthRadius = 6.371e6


def coord_cart(origin, point):
	
	error_factor = 1.37
	yval = (point[0] - origin[0]) * (np.pi / 180) * np.cos((point[1] - origin[1]) / 2 * np.pi / 180) * earthRadius
	xval = (point[1] - origin[1]) * (np.pi / 180) * earthRadius
	xval *= error_factor
	yval *= error_factor
	return (xval, yval)

def coord_distance(p1, p2):
	d = coord_cart(p1, p2)
	return m.sqrt(d[0] ** 2 + d[1] ** 2)
