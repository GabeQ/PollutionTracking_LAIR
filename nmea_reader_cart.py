import serial
from coord_cart_converter import coord_cart
from time import gmtime, strftime

# Select COM port to recieve data and open it
ser = serial.Serial()
ser.baudrate = 4800
ser.port = 'COM4'
ser.open()

# Create file to write cartesian coordinates to
file = open('coordinates4.txt', 'w+')

coord_list = []
origin = (34.075171, -117.434742)
# First line is junk, don't analyze it
bad_line = ser.readline()
while True:
	line = str(ser.readline())
	dat = list(line.split(','))

	if dat[0] == "b'$GPGGA":
		latitude = float(dat[2]) /100
		longitude = float(dat[4]) / -100
		coordinate = (latitude, longitude)
		cart_point = coord_cart(origin, coordinate)
		t = strftime("%H:%M:%S". gmtime())
		file.write(str(cart_point) + ' -- ' + str(t) + '\n')
		coord_list.append((cart_point, t))
		print(coord_list[-1])

	if dat[0] == "b'$GPGSA":
		continue

	if dat[0] == "b'$GPRMC":
		latitude = float(dat[3]) /100
		longitude = float(dat[5]) / -100
		coordinate = (latitude, longitude)
		cart_point = coord_cart(origin, coordinate)
		t = strftime("%H:%M:%S". gmtime())
		file.write(str(cart_point) + ' -- ' + str(t) + '\n')
		coord_list.append((cart_point, t))
		print(coord_list[-1])

	if dat[0] == "b'$GPGSV":
		continue

ser.close()
