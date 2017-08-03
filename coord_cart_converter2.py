import numpy as np
earthRadius = 6.371e6


def coord_cart(origin, point):
	
	xval = (point[0] - origin[0]) * (np.pi / 180) * np.cos((point[1] - origin[1]) / 2 * np.pi / 180) * earthRadius
	yval = (point[1] - origin[1]) * (np.pi / 180) * earthRadius
	return (xval, yval)
