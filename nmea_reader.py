import serial
#from coord_to_cart.py import set_cart_coords

ser = serial.Serial()
ser.baudrate = 4800
ser.port = 'COM4'
ser.open()

file = open('coordinates.txt', 'w+')

#s = (34.106187, -117.713174)
#graph = ox.graph_from_point(s, distance=500, network_type='walk')
#nodes = graph.nodes()
#set_cart_coords(graph, originCoord=s)

latOffset = .042457
longOffset = -.284534

bad_line = ser.readline()
coord_list = []
while len(coord_list) < 200:
	line = str(ser.readline())
	dat = list(line.split(','))

	if dat[0] == "b'$GPGGA":
		latitude = float(dat[2]) / 100
		longitude = float(dat[4]) / 100
		if dat[3] == 'S':
			latitude *= -1
		if dat[5] == 'W':
			longitude *= -1
		latitude += latOffset
		longitude += longOffset
		coordinate = (latitude, longitude)
		coord_list.append(coordinate)
		numSat = dat[7]
		file.write(str(coordinate)  +  '\n')
		print(coordinate)

	if dat[0] == "b'$GPGSA":
		continue

	if dat[0] == "b'$GPRMC":
		latitude = float(dat[3]) /100
		longitude = float(dat[5]) / 100
		if dat[4] == 'S':
			latitude *= -1
		if dat[6] == 'W':
			longitude *= -1
		latitude += latOffset
		longitude += longOffset
		coordinate = (latitude, longitude)
		coord_list.append(coordinate)
		file.write(str(coordinate) +  '\n')

	if dat[0] == "b'$GPGSV":
		continue


for point in coord_list:
	print(point)

ser.close()
