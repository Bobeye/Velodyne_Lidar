import dpkt
import socket
import math
import time
import numpy as np

import pcl

class READPCAP(object):
	def __init__(self, ROUTE, MODE):
		self.__route = ROUTE
		self.__mode = MODE
		self.__udp_sample_size = 1508
		self.__laser_id_n = 16
		self.__ares = 100	# 0.01 degree based
		self.__xres = 1000	# mm
		self.__yres = 1000	# mm


	def Reflectivity_to_RGBHex(self,FRR_reflectivity):
		
		rgb = (FRR_reflectivity,0,0)
		return '#%02x%02x%02x' % rgb

		return rgb

	def append_hex(self,a, b):
		return (a << 8) | b
		
	def append_time_hex(self,a,b):
		return (a << 8) | b

	def Spherical_to_xyz(self,r,omega,alpha):	# xyz in right-handed coordinates, y is the thumb
		Omega = math.radians(omega)
		Alpha = math.radians(alpha)
		x = r * math.cos(Omega) * math.sin(Alpha)
		y = r * math.cos(Omega) * math.cos(Alpha)
		z = r * math.sin(Omega)
		return [x, y, z]

	def Decode_UDP(self,FDU_udp):	# Decode udp packet coming through port 2368
		if len(FDU_udp) == 1248:
			pkt = FDU_udp
			eth = dpkt.ethernet.Ethernet(pkt)
			if eth.type == dpkt.ethernet.ETH_TYPE_IP :
				ip = eth.data
				if ip.p == dpkt.ip.IP_PROTO_UDP :
					# This doesn't deal with IP fragments                
					udp = ip.data
					# Pass the IP addresses, source port, destination port, and data back to the caller.
					return [ip.src, udp.sport, ip.dst, udp.dport, udp.data, ip.v]
			elif eth.type == dpkt.ethernet.ETH_TYPE_IP6 :
				ip = eth.data
				if ip.nxt == dpkt.ip.IP_PROTO_UDP :
					# This doesn't deal with IP fragments                
					udp = ip.data
					# Pass the IP addresses, source port, destination port, and data back to the caller.
					return [ip.src, udp.sport, ip.dst, udp.dport, udp.data, ip.v]
			else :
				# If the packet is something else, then I need to figure out a better way of handling it.
				return "ERROR"
		else:
			return "ERROR, length of the packet should be 1248"

	def Decode_Data_Packet(self,FDDP_data):	# decode 1248-byte data packet from UDP
		Mode = self.__mode
		if Mode == "Single" or Mode == "Dual":
			vertical_angle = [-15.0, 1.0, -13.0, -3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]		# vertical angle according to laser ID
			data_list = list(FDDP_data)
			data_len = len(data_list)
			data_blocks = list(range(13))	# 12 blocks + 1 time stamp in each data packet
			for i in range(13):
				data_blocks[i] = []
			# distribute data info to each block
			for i in range(data_len):
				data_blocks[i/100] = data_blocks[i/100] + [data_list[i]]
			time_block = data_blocks[12]
			data_blocks.pop(12)
			time_stamp_1 = self.append_time_hex(ord(time_block[3]) , ord(time_block[2]))
			time_stamp_2 = self.append_time_hex(ord(time_block[1]) , ord(time_block[0]))
			time_stamp = (time_stamp_1 << 16) | time_stamp_2		# micro-second
			# decode each block:
			decoded_data_blocks = list(range(12))
			for i in range(12):
				decoded_data_blocks[i] = self.Decode_Data_Block(data_blocks[i])
			
			# build Azimuth list
			Azimuth_list = list(range(12))
			for i in range(12):
				Azimuth_list[i] = decoded_data_blocks[i][0]
				decoded_data_blocks[i].pop(0)
				if i != 0:
					if Azimuth_list[i] < Azimuth_list[i-1]:
						Azimuth_list[i] = Azimuth_list[i] + 360
			if Mode == "Single": # either "Strongest" or "Last"
				Azimuth_list_single = list(range(24))
				for i in range(24):
					if i % 2 == 0:
						Azimuth_list_single[i] = Azimuth_list[i/2]
					elif i == 23:
						Azimuth_list_single[i] = Azimuth_list[11] + (Azimuth_list[11] - Azimuth_list[10]) / 2.0
					else:
						Azimuth_list_single[i] = Azimuth_list[i/2] + (Azimuth_list[(i+1)/2] - Azimuth_list[i/2]) / 2.0
				Azimuth_list_final = Azimuth_list_single
			else:
				Azimuth_list_dual = list(range(24))
				for i in range(6):
					if i == 5:
						Azimuth_list_dual[4*i] = Azimuth_list[i]
						Azimuth_list_dual[4*i+2] = Azimuth_list_dual[4*i]
						Azimuth_list_dual[4*i+1] = Azimuth_list[i] + (Azimuth_list[2*i] - Azimuth_list[2*i-2]) / 2.0
						Azimuth_list_dual[4*i+3] = Azimuth_list_dual[4*i+2]
					else:
						Azimuth_list_dual[4*i] = Azimuth_list[i]
						Azimuth_list_dual[4*i+2] = Azimuth_list_dual[4*i]
						Azimuth_list_dual[4*i+1] = Azimuth_list[i] + (Azimuth_list[2*i+2] - Azimuth_list[2*i]) / 2.0
						Azimuth_list_dual[4*i+3] = Azimuth_list_dual[4*i+2]
				Azimuth_list_final = Azimuth_list_dual
			for i in range(24):
				if Azimuth_list_final[i] > 360:
					Azimuth_list_final[i] = Azimuth_list_final[i] % 360
			
			# build data points without time stamp
			# [block id (0-11),
			#  firing sequence (1-2),
			#  laser ID (0-15),
			#  Cartesian Coordinates x
			#  Cartesian Coordinates y
			#  Cartesian Coordinates z
			#  reflectivity (0-255)
			#  Distance mm
			#  Azimuth (0-355.99)
			#  (time stamp)
			data_points = list(range(384)) # 12 blocks * (16 lasers * 2 firing sequences) = 384 data points in data block
			for n in xrange(384):
				data_points[n] = list(range(9))
			n = 0
			for i in xrange(12):
				for j in xrange(32):
					data_points[n][0] = i 	# block id (0-11)
					data_points[n][1] = (j/16) + 1	# firing sequence (1-2)
					data_points[n][2] = j%16 	#  laser ID (0-15)
					data_points[n][6] = decoded_data_blocks[i][j*2+1] #  reflectivity (0-255)
					data_points[n][7] = decoded_data_blocks[i][j*2]	#  Distance mm
					#  Azimuth (0-355.99)		
					if data_points[n][1] == 1:
						data_points[n][8] = Azimuth_list_final[i*2]
					else:
						data_points[n][8] = Azimuth_list_final[i*2+1]
					cartesian_coordinates = self.Spherical_to_xyz(data_points[n][7], vertical_angle[data_points[n][2]], data_points[n][8])
					data_points[n][3] = cartesian_coordinates[0]
					data_points[n][4] = cartesian_coordinates[1]
					data_points[n][5] = cartesian_coordinates[2]
					n += 1
			# add time stamps
			for i in range(384):
				sequence_index = data_points[i][1] + 2 * data_points[i][0] - 1
				data_points[i].append( time_stamp 
									  +((55.296 * sequence_index))
									  +((2.304 * data_points[i][2])))

			return data_points
		else:
			return "ERROR, wrong mode"

	def Decode_Data_Block(self,FDDB_data_block):	# decode 1200-byte data block + 4-byte time block + 2-byte factory block 
		data_block = FDDB_data_block
		# Azimuth
		azimuth = self.append_hex(ord(data_block[3]),ord(data_block[2])) / 100.0
		# Location: Distance & reflectivity
		location = list(range(64))	# (1 distance + 1 reflectivity) * 32 channels = 64 location info in one block
		location_n = 0
		for i in range(4,98):
			if i % 3 == 1:
				distance = self.append_hex(ord(data_block[i+1]),ord(data_block[i])) * 2.0	# mm
				reflectivity = ord(data_block[i+2])
				location[location_n] = distance
				location[location_n+1] = reflectivity
				location_n += 2
		decoded_data_block = [azimuth] + location
		return decoded_data_block

	def Decode_Pcap(self,FDP_pcap, FDP_sample_size):	# decode parsed pcap file with pre-defined number of packets
		Mode = self.__mode
		udp_list = FDP_pcap
		sample_size = FDP_sample_size
		raw_decoded_udp = list(range(sample_size))
		raw_decoded_udp_data = list(range(sample_size))
		for i in xrange(sample_size):
			raw_decoded_udp[i] = self.Decode_UDP(udp_list[i])[4]
			raw_decoded_udp_data[i] = self.Decode_Data_Packet(raw_decoded_udp[i])
		# basic filter for irrelevant points
		decoded_data_list = [0]
		for i in xrange(sample_size):
			for j in range(len(raw_decoded_udp_data[i])):
				if raw_decoded_udp_data[i][j][7] <= 1 or raw_decoded_udp_data[i][j][7] >= 150000:
					pass
				else:
					decoded_data_list.append(raw_decoded_udp_data[i][j])
		decoded_data_list.pop(0)

		return decoded_data_list 	

	def Read_Pcap(self):
		FRP_pcap = self.__route
		Mode = self.__mode
		pcap_file = open(FRP_pcap)				# import the .pcap file
		pcap = dpkt.pcap.Reader(pcap_file)		# read the .pcap file using dpkt
		# save the .pcap to buffer
		buf_list = [0]
		for ts, buf in pcap:
			buf_list.append(buf)
		buf_list.pop(0)
		packet_num = len(buf_list)	# number of packets in the pcap file

		print ("number of total packets: ", packet_num)

		udp_list = [0]	# initialize the list of all udp packets
		gps_list = [0]	# initialize the list of all gps packets
		for i in range(packet_num):
			if len(buf_list[i]) == 1248:	# udp packet
				udp_list.append(buf_list[i])
			elif len(buf_list[i]) == 554:	# position(GPS) packet
				gps_list.append(buf_list[i])
			else:
				pass
		udp_list.pop(0)
		gps_list.pop(0)
		# crack udp list
		udp_num = len(udp_list)		# number of udp packets

		print("number of udp packets: ", udp_num)

		return udp_list

	def PCAP(self):
		udp_list = self.Read_Pcap()
		udp_len = len(udp_list)

		# decode selected number of upd packets:
		sample_size = self.__udp_sample_size
		ul = udp_list[6500:6500+sample_size]
		dul = self.Decode_Pcap(ul, sample_size)

		return dul


		# initialize the environment with selected number of udp packets
		# fit all the decoded points into a matrix with laserID x azimuth
		"""
		In:
		[block id (0-11),
		 firing sequence (1-2),
		 laser ID (0-15),
		 Cartesian Coordinates x
		 Cartesian Coordinates y
		 Cartesian Coordinates z
		 reflectivity (0-255)
		 Distance mm
		 Azimuth (0-355.99)
		 time stamp]
		Out:
		 nest_pts_ida : [number of points, x,y,z,reflectivity,distance,azimuth, laser id]
		 # nest_pts_xy : [number of points, averagex, averagey, averagez ]

		points classification id: [ida_id, xy_id, ...]

		"""
		# ptsn = len(dul)
		# idn = self.__laser_id_n
		# ares = self.__ares
		# an = int(36000 / ares)
		# nest_pts_ida = np.zeros([idn*an, 8], dtype = np.float64)
		# nest_pts_pid = list(range(ptsn))
		# for i in xrange(len(dul)):
		# 	laserID = dul[i][2]
		# 	azimuth = dul[i][8]
		# 	ix = laserID * an + (int(azimuth*100) / ares)
		# 	nest_pts_pid[i] = [ix]
		# 	if nest_pts_ida[ix][0] >= 1:
		# 		nest_pts_ida[ix][1] = (nest_pts_ida[ix][1] * nest_pts_ida[ix][0] + dul[i][3])/(nest_pts_ida[ix][0]+1)
		# 		nest_pts_ida[ix][2] = (nest_pts_ida[ix][2] * nest_pts_ida[ix][0] + dul[i][4])/(nest_pts_ida[ix][0]+1)
		# 		nest_pts_ida[ix][3] = (nest_pts_ida[ix][3] * nest_pts_ida[ix][0] + dul[i][5])/(nest_pts_ida[ix][0]+1)
		# 		nest_pts_ida[ix][4] = (nest_pts_ida[ix][4] * nest_pts_ida[ix][0] + dul[i][6])/(nest_pts_ida[ix][0]+1)
		# 		nest_pts_ida[ix][5] = (nest_pts_ida[ix][5] * nest_pts_ida[ix][0] + dul[i][7])/(nest_pts_ida[ix][0]+1)
		# 		nest_pts_ida[ix][6] = (nest_pts_ida[ix][6] * nest_pts_ida[ix][0] + dul[i][8])/(nest_pts_ida[ix][0]+1)
		# 		nest_pts_ida[ix][0] += 1
		# 	else:
		# 		nest_pts_ida[ix][0] = 1
		# 		nest_pts_ida[ix][1:7] = dul[i][3:9]
		# 	nest_pts_ida[ix][7] = laserID			

		# xres = self.__xres
		# yres = self.__yres
		# xn = int(150000 / xres) * 2
		# yn = int(150000 / yres) * 2
		# nest_pts_xy = np.empty([xn*yn, 4], dtype = np.float64)
		# for i in xrange(len(dual)):
		# 	xc = int(dul[i][3]) / xres
		# 	yc = int(dul[i][4]) / yres

		# nest_pts_ida = nest_pts_ida[~np.all(nest_pts_ida == 0, axis=1)]


		# return [nest_pts_pid, nest_pts_ida, dul] 





if __name__=='__main__':


	# Read_Pcap('/home/poopeye/Desktop/WorkRelated/lidar/VELODYNE/VLP-16 Sample Data/2015-07-23-14-37-22_Velodyne-VLP-16-Data_Downtown 10Hz Single.pcap', "Single")

	# Read_Pcap('/home/poopeye/Desktop/WorkRelated/02-Marrysville-2016-09-13-14-18-15_Velodyne-VLP-16-Data.pcap', "Dual")

	mypcap = READPCAP('/home/bowen/Desktop/lidar/MovingVehicleRecord/2016-09-28-14-35-03_Velodyne-VLP-16-Data-Side.pcap', "Single")
	nest_pts_info = mypcap.PCAP()

	id_pts = nest_pts_info[0]
	nest_pts = nest_pts_info[1]
	origin_pts = nest_pts_info[2]

	print nest_pts


	start = time.time()
	pcl_pts = pcl.PointCloud(nest_pts, id_pts)
	tag_pcl_pts = pcl_pts.point_tag()
	end = time.time()

	print ("detection time: ", end-start)




	import matplotlib.pyplot as plt
	import mpl_toolkits.mplot3d.axes3d as p3


	# fig = plt.figure()
	# test_pcl_plot = fig.gca(projection='3d')
	# test_pcl_plot.set_xlim3d(-200000, 200000)
	# test_pcl_plot.set_ylim3d(-200000, 200000)
	# test_pcl_plot.set_zlim3d(-200000, 200000)
	# x_array = []
	# y_array = []
	# z_array = []
	# for i in xrange(tag_pcl_pts.shape[1]):
	# 	if tag_pcl_pts[0][i] == 0:
	# 		x_array += [nest_pts[i][1]]
	# 		y_array += [nest_pts[i][2]]
	# 		z_array += [nest_pts[i][3]]
	# test_pcl_plot.plot(x_array,y_array,z_array,'.')
	# plt.show()

	fig = plt.figure()
	test_pcl_plot = fig.gca(projection='3d')
	test_pcl_plot.set_xlim3d(-200000, 200000)
	test_pcl_plot.set_ylim3d(-200000, 200000)
	test_pcl_plot.set_zlim3d(-200000, 200000)
	x_array = []
	y_array = []
	z_array = []
	for i in xrange(len(origin_pts)):
		x_array += [origin_pts[i][3]]
		y_array += [origin_pts[i][4]]
		z_array += [origin_pts[i][5]]
	test_pcl_plot.plot(x_array,y_array,z_array,'.')
	plt.show()


