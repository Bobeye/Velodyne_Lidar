import dpkt
import socket
import math
import time
import numpy as np


class PcaptoPoints(object):
	def __init__(self, MODE):
		self.__mode = MODE
		self.__init_sample_num = 1500

	def Read_Udp(self):
		mode = self.__mode
		init_sample_num = self.__init_sample_num

		UDP_IP = "192.168.1.77"
		UDP_PORT = 2368

		sock = socket.socket(socket.AF_INET, # Internet
	                     socket.SOCK_DGRAM) # UDP
		sock.bind(('', UDP_PORT))
		
		for n in xrange(1000):
			buf_l = [0]
			for i in xrange(sample_size):
				data, addr = sock.recvfrom(1248) # buffer size is 1248 bytes
				buf_l.append(data)
			buf_l.pop(0)
			print ("new udp list")
			udp_list = buf_l
			Mode = "Single"


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


	UDP_IP = "192.168.1.77"
	UDP_PORT = 2368

	sock = socket.socket(socket.AF_INET, # Internet
	                     socket.SOCK_DGRAM) # UDP
	sock.bind(('', UDP_PORT))

	sample_size = 100

	plt.ion()
	sample_fig = plt.figure()
	sample_plot = sample_fig.gca(projection = '3d')
	

	
	for n in xrange(1000):
		
		buf_l = [0]
		for i in xrange(sample_size):
			data, addr = sock.recvfrom(1248) # buffer size is 1248 bytes
			buf_l.append(data)
		buf_l.pop(0)
		print ("new udp list")
		udp_list = buf_l
		Mode = "Single"