import serial
from coord_cart_converter2 import coord_cart
from time import gmtime, strftime
import csv

MCPC_filename = 'MCPC_170802_142909.txt'

# Create header row for MCPC data
with open(MCPC_filename) as f:
	reader = csv.reader(f)
	header_dat = list(reader)
	header_row = header_dat[14]

# Select COM port to recieve data and open it
ser = serial.Serial()
ser.baudrate = 4800
ser.port = 'COM5'
ser.open()

# Create file to write cartesian coordinates to
nmea_file = open('coordinates4.txt', 'w+')

origin = (34.075171, -117.434742)
# First line of nmea data is junk, don't analyze it
bad_line = ser.readline()

MCPC_data = []
coord_list = []
while True:

	#MCPC code
	with open(MCPC_filename) as f:
		reader = csv.reader(f)
		dat_file = list(reader)

	if dat_file[-1][0] != '#':
		current = dat_file[-1][0]
		line = current.split('\t')
		if not MCPC_data or line != MCPC_data[-1]:
			MCPC_data.append(line)
			print(MCPC_data[-1])

	#nmea code
	nmea_line = str(ser.readline())
	nmea_dat = list(nmea_line.split(','))

	if nmea_dat[0] == "b'$GPGGA":
		latitude = float(nmea_dat[2]) /100
		longitude = float(nmea_dat[4]) / -100
		coordinate = (latitude, longitude)
		cart_point = coord_cart(origin, coordinate)
		t = strftime("%H:%M:%S", gmtime())
		nmea_file.write(str(cart_point) + ' -- ' + str(t) + '\n')
		coord_list.append((cart_point, t))
		print(coord_list[-1])

	if nmea_dat[0] == "b'$GPGSA":
		continue

	if nmea_dat[0] == "b'$GPRMC":
		latitude = float(nmea_dat[3]) /100
		longitude = float(nmea_dat[5]) / -100
		coordinate = (latitude, longitude)
		cart_point = coord_cart(origin, coordinate)
		t = strftime("%H:%M:%S", gmtime())
		nmea_file.write(str(cart_point) + ' -- ' + str(t) + '\n')
		coord_list.append((cart_point, t))
		print(coord_list[-1])

	if nmea_dat[0] == "b'$GPGSV":
		continue

ser.close()