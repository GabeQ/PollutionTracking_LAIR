#!/usr/bin/env python

'''kalmanFiltering.py: Contains functions to perform the Kalman Filter. Cite http://scipy-cookbook.readthedocs.io/items/KalmanFiltering.html'''

__author__ = "Gabriel Quiroz"
__copyright__ = "Copyright 2017, LAIR Project"

import random


def particle_sensor_sim():
	'''Generates random values that simulate a particle sensor getting measurements'''
	return random.randint(30, 70)

def meas_var_dist(distance):
	'''calculates the measured value variance for a given point given
	the variance is linear with respect to distance'''
	var = (1/5)*distance
	return var


def kalman_filter_mult_cycles(true_val, estimate, estimate_var, measured_var, num_cycl):
	'''uses the kalman filter to come up with estimated values of something being measured. Runs the
	filter based on the amount of cycles chosen.'''

	sz = (num_cycl,)		# size of array
	x = true_val			# truth value (typo in example at top of p. 13 calls this z)
	z = np.random.normal(x,0.1,size=sz) # observations (normal about x, sigma=0.1)

	Q = 1e-5 				# process variance

	# allocate space for arrays
	xhat=np.zeros(sz)      	# a posteri estimate of x
	P=np.zeros(sz)         	# a posteri error estimate
	xhatminus=np.zeros(sz) 	# a priori estimate of x
	Pminus=np.zeros(sz)    	# a priori error estimate
	KG=np.zeros(sz)         	# gain or blending factor

	R = measured_var 		# estimate of measurement variance, change to see effect

	# intial guesses
	xhat[0] = estimate
	P[0] = estimate_var

	for k in range(1,num_cycl):
	    # time update
	    xhatminus[k] = xhat[k-1]
	    Pminus[k] = P[k-1]+Q

	    # measurement update
	    KG[k] = Pminus[k]/( Pminus[k]+R )
	    xhat[k] = xhatminus[k]+KG[k]*(z[k]-xhatminus[k])
	    P[k] = (1-KG[k])*Pminus[k]

	return xhat


def kalman_filter(priori_est, priori_estVar, meas_val, meas_valVar):
	'''Performs the Kalman Filter algorithm for 1 cycle, returns the posteri estimate
	and variance for the next cycle'''
	KG = priori_estVar/(priori_estVar + meas_valVar)
	posteri_est = priori_est + KG*(meas_val - priori_est)
	posteri_estVar = (1-KG)*priori_estVar
	return posteri_est, posteri_estVar


def kalman_filter_routing(graph, est, est_var, meas_var, startNode, num_cycl):
	'''Takes graph with node pollution, an estimated variance, and a start node, and routes to other nodes
	on the graph based on their predicted pollution estimates gathered from the Kalman Filter'''
	largestEst = 0
	targetNode = 0

	for n in graph.nodes():
		estimated = kalman_filter(graph.node[n]['pollution_amount'], est, est_var, meas_var, num_cycl)[-1]
		if estimated > largestEst:
			largestEst = estimated
			targetNode = n

	return nx.shortest_path(graph, startNode, targetNode, weight = 'length')
