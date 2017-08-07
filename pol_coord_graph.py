import matplotlib.pyplot as plt
import numpy as np
import csv
nmea_filename = 'test_data\campus_test_2\campus_coords_170804.txt'
mcpc_filename = 'test_data\campus_test_2\MCPC_170804_103336.txt'

# open nmea data and convert to str list
with open(nmea_filename) as f:
	reader = csv.reader(f)
	nmea_file = list(reader)

# seperate coordinates and time into seperate lists, and convert str to float
x_c = []
y_c = []
nmea_time = []
for point in nmea_file:
	x_c.append(float(point[0][1:]))
	second = point[1].split(')')
	y_c.append(float(second[0][1:]) + 24228)
	nmea_time.append(second[1][4:])

# adjust for GPS time error
for i in range(len(nmea_time)):
	t = nmea_time[i].split(':')
	t[0] = str(int(t[0]) - 7)
	nmea_time[i] = (t[0] + ':' + t[1] + ':' + t[2])

# put coordinates and time usable list
nmea_data = []
for i in range(len(nmea_file)):
	nmea_data.append(((x_c[i], y_c[i]), nmea_time[i]))

# open mcpc data
with open(mcpc_filename) as f:
	reader = csv.reader(f)
	mcpc_file = list(reader)

mcpc_data = []
for line in mcpc_file:
	dat = line[0].split('\t')
	mcpc_data.append(dat)
mcpc_data = mcpc_data[15:]

total_data = []
for point in mcpc_data:
	time = point[1]
	match = -1
	for i in range(len(nmea_data)):
		if nmea_data[i][1] == time:
			match = i
			continue
	if match != -1:
		total_data.append([point, nmea_data[match]])

x_coords = [point[1][0][0] for point in total_data]
y_coords = [point[1][0][1] for point in total_data]
pollution = [point[0][3] for point in total_data]

graph = plt.scatter(x_coords, y_coords, c=pollution)
cb = plt.colorbar(graph)

plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
cb.set_label('Coincidence Corrected Concentration (#/' + r'$cm^3$' + ')')

plt.show()