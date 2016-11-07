import dpkt
import socket
import math
import time
import numpy as np
import numbapro
from numbapro import guvectorize, vectorize, cuda

class PcaptoPoints(object):
	def __init__(self, ROUTE, MODE):
		self.__route = ROUTE
		self.__mode = MODE

	def Read_Pcap(self):
		Route = self.__route
		Mode = self.__mode
		pcap_file = open(Route)				# import the .pcap file
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

		return [udp_list, gps_list]


class RealTimePointCloud(object):
	def __init__(self, UDP, GPS, MODE):
		self.__udp = UDP 	# udp packets
		self.__gps = GPS 	# gps packets
		self.__mode = MODE
		if MODE == 'Single':
			self.__un = 754 	# 754 packets per second for single mode
		else:
			self.__un = 1508
		self.__laser_angle = [-15.0, 1.0, -13.0, -3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]		# vertical angle according to laser ID

	def PointCloud_init(self):
		udps = self.__udp
		mode = self.__mode
		un = self.__un
		laser_angle = self.__laser_angle

		# initialize the point cloud environment with the first 'un' udp packets
		# register each point to the narray as 16(laser if) * 3600 (per 0.1 degree)   
		pts = np.zeros([16*3600, 10], dtype = np.int64)
		for u in xrange(un):
			ptsc =  [] # 12 blocks * (16 lasers * 2 firing sequences) = 384 data points in one udp packet
			udpc = udps[u] 		# current udp packet
			eth = dpkt.ethernet.Ethernet(udpc)
			# extract data packet from current udp
			if eth.type == dpkt.ethernet.ETH_TYPE_IP :
				ip = eth.data
				if ip.p == dpkt.ip.IP_PROTO_UDP :
					# This doesn't deal with IP fragments                
					udp = ip.data
					# Pass the IP addresses, source port, destination port, and data back to the caller.
					# decode_udpc = [ip.src, udp.sport, ip.dst, udp.dport, udp.data, ip.v]
					dp = list(udp.data)
			elif eth.type == dpkt.ethernet.ETH_TYPE_IP6 :
				ip = eth.data
				if ip.nxt == dpkt.ip.IP_PROTO_UDP :
					# This doesn't deal with IP fragments                
					udp = ip.data
					# Pass the IP addresses, source port, destination port, and data back to the caller.
					# decode_udpc = [ip.src, udp.sport, ip.dst, udp.dport, udp.data, ip.v]
					dp = list(udp.data)
			# decode the data packets
			data_blocks = list(range(13))
			for d in range(13):		# initialize an empty list of data blocks for single udp packet (12 data blocks + 1 time block)
				data_blocks[d] = []
			for d in xrange(len(dp)):
				data_blocks[d/100] = data_blocks[d/100] + [dp[d]]
			time_block = data_blocks[12]
			data_blocks.pop(12)

			# decide the time
			time_stamp_1 = ((ord(time_block[3]))<<8) | (ord(time_block[2]))
			time_stamp_2 = ((ord(time_block[1]))<<8) | (ord(time_block[0]))
			time_stamp = (time_stamp_1 << 16) | time_stamp_2		# micro-second

			# decode each data block:
			azimuth = []
			for db in xrange(12):
				azimuth +=  [(((ord(data_blocks[db][3]))<<8) | (ord(data_blocks[db][2])))]
				if db > 0 and azimuth[db] < azimuth[db-1]:
					azimuth[db] += 36000
				for i in xrange(4,98):
					if i % 3 == 1:
						distance = (((ord(data_blocks[db][i+1]))<<8) | (ord(data_blocks[db][i]))) * 2.0		# mm
						reflectivity = ord(data_blocks[db][i+2])
						block_id = db
						f_sequence = (((db+1)*32) + (((i-4)/3)+1) -1)  % 32 / 16 + 1
						laser_id = (((db+1)*32) + (((i-4)/3)+1) - 1) % 16
						laser_a = laser_angle[ laser_id ]
						sequence_index = f_sequence+2*block_id-1
						time_current = int(time_stamp+((55.296 * sequence_index))+((2.304 * laser_id)))  	# micro second
						# [time, block id(0-11), firing sequence(1-2), laser id(0-15), laser angle(degree), reflectivity, distance]
						ptsc += [[time_current, db, f_sequence, laser_id, laser_a, reflectivity, distance]]

			# generate the azimuth dict for the current data packet
			azimuth_dict = list(range(24))
			for i in xrange(24):
				if i % 2 == 0:
					azimuth_dict[i] = azimuth[i/2]
				elif i == 23:
					azimuth_dict[i] = azimuth[11] + (azimuth[11] - azimuth[10]) / 2.0
				else:
					azimuth_dict[i] = azimuth[(i-1)/2] + (azimuth[(i-1)/2+1] - azimuth[(i-1)/2]) / 2.0
				if azimuth_dict[i] > 36000:
					azimuth_dict[i] = azimuth_dict[i] % 36000
			for i in xrange(384):				
				ac = int(azimuth_dict[i/24] / 10)
				lc = ptsc[i][3]
				depth = ptsc[i][6] * math.cos(math.radians(ptsc[i][4])) * math.sin(math.radians(ac/10.0))
				height = ptsc[i][6] * math.sin(math.radians(ptsc[i][4]))
				if depth != 0:
					pts[lc*3600+ac] = np.array(ptsc[i] + [ac, abs(depth), height])
				else:
					pts[lc*3600+ac] = np.array(ptsc[i] + [ac, 00000, 00000])

		
		"""
		pts structure:
		[	00 time(micro second)
			01 block id(0-11)
			02 firing sequence(1-2)
			03 laser ID(0-15)
			04 laser angle(degree)
			05 reflectivity(0-255)
			06 distance(mm)
			07 azimuth(degree)
			08 depth(mm)
			09 height(mm)
		]
		"""

		depth_pts = (pts.transpose()[[3,7,8]]).transpose()
		height_pts = (pts.transpose()[[3,7,9]]).transpose()

		Point_Cloud_Init = {	'points' : pts,
								'depth'  : depth_pts,
								'height' : height_pts }

		return Point_Cloud_Init
			


if __name__=='__main__':
	packets = PcaptoPoints('/home/bowen/Desktop/lidar/Cyber_Lab_Study_Sample/2016-09-20-13-12-25_Velodyne-VLP-16-Data-Dual.pcap', "Dual").Read_Pcap()
	udp = packets[0]
	gps = packets[1]
	
	t1 = time.time()
	pc= RealTimePointCloud(udp,gps,'Dual')
	pts = pc.PointCloud_init()
	t2 = time.time()
	print t2-t1

	
	# print pts
	origin_pts = pts['points']
	depth_pts = pts['depth']
	height_pts = pts['height']
	# print np.amax(height_pts.transpose()[[2]])
	# print np.amin(height_pts.transpose()[[2]])
	n = np.zeros(16)
	m = np.zeros(3600)
	for i in xrange(16*3600):
		if depth_pts[i][2] == 0:
			n[depth_pts[i][0]] += 1
			m[depth_pts[i][1]] += 1

	print m
	print n

	# [  74.   135.    136.   98.    186.   93.   299.   51.    256.   42.    393.   82.    314.   50.   302.   48.]
	# [ 2819.  2275.  2513.  2724.  1886.  2880.  1430.  3150.  1061.  3539.  910.  3598.   833.  3598.  2139.  3598.]
	#  [-15.0, 1.0,   -13.0, -3.0,  -11.0, 5.0,   -9.0,  7.0,   -7.0,  9.0,  -5.0,  11.0,   -3.0, 13.0, -1.0,   15.0]






	# def make_cmap(colors, position=None, bit=False):
	# 	'''
	# 	make_cmap takes a list of tuples which contain RGB values. The RGB
	# 	values may either be in 8-bit [0 to 255] (in which bit must be set to
	# 	True when called) or arithmetic [0 to 1] (default). make_cmap returns
	# 	a cmap with equally spaced colors.
	# 	Arrange your tuples so that the first color is the lowest value for the
	# 	colorbar and the last is the highest.
	# 	position contains values from 0 to 1 to dictate the location of each color.
	# 	'''
	# 	import matplotlib as mpl
	# 	import numpy as np
	# 	bit_rgb = np.linspace(0,1,256)
	# 	if position == None:
	# 	    position = np.linspace(0,1,len(colors))
	# 	else:
	# 	    if len(position) != len(colors):
	# 	        sys.exit("position length must be the same as colors")
	# 	    elif position[0] != 0 or position[-1] != 1:
	# 	        sys.exit("position must start with 0 and end with 1")
	# 	if bit:
	# 	    for i in range(len(colors)):
	# 	        colors[i] = (bit_rgb[colors[i][0]],
	# 	                     bit_rgb[colors[i][1]],
	# 	                     bit_rgb[colors[i][2]])
	# 	cdict = {'red':[], 'green':[], 'blue':[]}
	# 	for pos, color in zip(position, colors):
	# 	    cdict['red'].append((pos, color[0], color[0]))
	# 	    cdict['green'].append((pos, color[1], color[1]))
	# 	    cdict['blue'].append((pos, color[2], color[2]))

	# 	cmap = mpl.colors.LinearSegmentedColormap('my_colormap',cdict,256)
	# 	return cmap

	# import matplotlib.pyplot as plt
	# x = height_pts.transpose

	# plt.plot()



	# pt = np.zeros(16*3600)
	# pt = pt.reshape((16,3600))
	# colors = list(range(16*3600))
	# for n in xrange(16*3600):
	# 	colors[n] = (1-abs(pts[n][8])/130000.0,1-pts[n][8]/130000.0,1-pts[n][8]/130000.0)
	# plt.pcolor(depth_pts, cmap=make_cmap(colors))
	# plt.colorbar()
	# plt.show()



	# ax = fig.add_subplot(313)
	# colors = [(0.4,0.2,0.0), (1,1,1), (0,0.3,0.4)]
	# ### Create an array or list of positions from 0 to 1.
	# position = [0, 0.3, 1]
	# plt.pcolor(np.random.rand(25,50), cmap=make_cmap(colors, position=position))
	# plt.colorbar()



	# import mpl_toolkits.mplot3d.axes3d as p3
	# fig = plt.figure()
	# test_pcl_plot = fig.gca(projection='3d')
	# test_pcl_plot.set_xlim3d(-1, 3660)
	# test_pcl_plot.set_ylim3d(0, -20000)
	# test_pcl_plot.set_zlim3d(-16, 16)
	# x_array = []
	# y_array = []
	# z_array = []
	# for pt in pts:
	# 	if pt[8] != 0:
	# 	 # and pt[9] >= -1000 and pt[6] >= 50000:
	# 		x_array += [pt[7]]
	# 		y_array += [-pt[8]]
	# 		z_array += [pt[4]]
	# test_pcl_plot.plot(x_array,y_array,z_array,'.')

	# plt.show()

