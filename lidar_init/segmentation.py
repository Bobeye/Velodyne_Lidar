import dpkt
import socket
import math
import time
import numpy as np
# import numbapro
# from numbapro import guvectorize, vectorize, cuda

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy import ndimage

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler

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
		self.__laser_angle = [-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]		# vertical angle according to laser ID

	def PointCloud_init(self):
		udps = self.__udp
		mode = self.__mode
		laser_angle = self.__laser_angle

		pts = []

		for u in xrange(len(udps)):
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
				ac = int(azimuth_dict[i/24] / 100)-1
				lc = ptsc[i][3]
				depth = ptsc[i][6] * math.cos(math.radians(ptsc[i][4])) * math.sin(math.radians(ac/100.0))
				height = ptsc[i][6] * math.sin(math.radians(ptsc[i][4]))
				if depth != 0:
					temp_pts =  np.array(ptsc[i] + [ac, abs(depth), height])
					pts += [temp_pts]


		
		"""
		pts structure:
		[	00 time(micro second)
			01 block id(0-11)
			02 firing sequence(1-2)
			03 laser ID(0-15)
			04 laser angle(degree)
			05 reflectivity(0-255)
			06 distance(mm)
			07 azimuth(0.1 degree)
			08 depth(mm)
			09 height(mm)
		]
		"""

		Point_Cloud_Init = {	'points' : pts }


		return Point_Cloud_Init

class ObjectDetection(object):
	def __init__(self, route):
		self.__route = route
		self.__blind_dist = 200 # / math.sin(np.deg2rad(15)) 

	def PointCloud(self):
		route = self.__route
		for f in xrange(3):
			filename = route + str(100019 + f) + '.pcap'
			packets = PcaptoPoints(filename, 'Dual').Read_Pcap()
			udp = packets[0][0:900]
			gps = packets[1]
			
			pcl = RealTimePointCloud(udp,gps,'Dual')
			pts = pcl.PointCloud_init()
			print (str(len(pts['points'])) + ' points extracted from ' + str(100000 + f) + '.pcap')
			self.ObjectCluster(pts['points'])

	def ObjectCluster(self, ptss):
		bd = self.__blind_dist

		# find the round
		azm_init = ptss[0][7]
		last_azm_index = 0
		for pn in xrange(len(ptss)-1):
			if ptss[pn][7]-azm_init<0 and ptss[pn+1][7]-azm_init>=0:
				opts = ptss[last_azm_index:pn+1]
				last_azm_index = pn+1
				print ('number of points for single round scan: ' + str(len(opts)))

				# arrange the points with laser angle form bottom to top
				pts = np.empty([16*360,10]).astype(np.int64)
				for i in xrange(len(opts)):
					if opts[i][3] % 2 == 0:
						pts[(opts[i][3] / 2)*360+(i%360)] = opts[i]
						pts[(opts[i][3] / 2)*360+(i%360)][1] = i	# replace block id with pid
					else:
						pts[((opts[i][3]+15)/2)*360+(i%360)] = opts[i]
						pts[((opts[i][3]+15)/2)*360+(i%360)][1] = i	# replace block id with pid

				# clean up points
				for i in xrange(16*360):
					if pts[i][6] > 20000: 		# distance > 20m
						pts[i][6] = 0
						pts[i][8] = 0
						pts[i][9] = 0
				
				# remove ground truth	
				for i in xrange(360):
					j = 0
					gt_stop = False
					gt_1st = 0
					while j < 16 and not gt_stop:
						n = i + (360*j)
						if gt_1st == 0:
							if pts[n][6] > bd:
								gt_1st = pts[n][9]	# first ground truth point
								pts[n][6] = 0
								pts[n][8] = 0
								pts[n][9] = 0
						else:
							for ni in [-3,-2,-1,0,1,2,3]:
								n += ni
								if n >= 0 and n <360:
									h_dif = abs(pts[n][9]-gt_1st)
									if h_dif <= 500:
										pts[n][6] = 0
										pts[n][8] = 0
										pts[n][9] = 0
								else:
									pass
						j += 1

				# xyz analysis
				pts_x = np.empty([16,360]).astype(np.int64)
				pts_y = np.empty([16,360]).astype(np.int64)
				pts_z = np.empty([16,360]).astype(np.int64)
				for i in xrange(16):
					for j in xrange(360):
						n = 360*i+j
						pts_x[i][j] = pts[n][6] * math.cos(pts[n][4]/360.0*(2*math.pi)) * math.sin(pts[n][7]/360.0*(2*math.pi))
						pts_y[i][j] = pts[n][6] * math.cos(pts[n][4]/360.0*(2*math.pi)) * math.cos(pts[n][7]/360.0*(2*math.pi))
						pts_z[i][j] = pts[n][6] * math.sin(pts[n][4]/360.0*(2*math.pi))
				pts_x = pts_x.reshape([16*360,])
				pts_y = pts_y.reshape([16*360,])
				pts_z = pts_z.reshape([16*360,])
				pts_xyz = np.array([pts_x,pts_y,pts_z]).T
				pts_xyz = pts_xyz[~np.all(pts_xyz == 0, axis=1)]
				pts_xy = np.delete(pts_xyz,-1,1)

				# # rough cluster -- kd-tree
				# DIM = 3
				# from scipy.spatial import KDTree
				# tree = KDTree(pts_xyz, leafsize=pts_xyz.shape[0]+1)


				# DBSCAN
				db = DBSCAN(eps=300, min_samples=20).fit(pts_xy)
				core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
				core_samples_mask[db.core_sample_indices_] = True
				labels = db.labels_

				# Number of clusters in labels, ignoring noise if present.
				n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

				print('Estimated number of clusters: %d' % n_clusters_)

				fig = plt.figure()
				pts_plot = fig.gca(projection = '3d')
				pts_plot.set_xlim3d(-15000,15000)
				pts_plot.set_ylim3d(-15000,15000)
				pts_plot.set_zlim3d(-15000,15000)
				
				unique_labels = set(labels)
				colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
				for k, col in zip(unique_labels, colors):
				    if k == -1:
				        # Black used for noise.
				        col = 'k'

				    class_member_mask = (labels == k)

				    xyz = pts_xyz[class_member_mask & core_samples_mask]
				    plt.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], 'o', markerfacecolor=col,
				             markeredgecolor='k', markersize=5)

				    xyz = pts_xyz[class_member_mask & ~core_samples_mask]
				    plt.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], 'o', markerfacecolor=col,
				             markeredgecolor='k', markersize=2)
				
				plt.show()






				# fig = plt.figure()
				# pts_plot = fig.gca(projection = '3d')
				# pts_plot.set_xlim3d(-15000,15000)
				# pts_plot.set_ylim3d(-15000,15000)
				# pts_plot.set_zlim3d(-15000,15000)
				# xo = []
				# yo = []
				# zo = []
				# for pt in pts_xyz:
				# 	xo += [pt[0]]
				# 	yo += [pt[1]]
				# 	zo += [pt[2]]

				# pts_plot.plot(xo,yo,zo,'g.')	
				
				# plt.show()









class PedestrianClass(object):

	def __init__(self, route):
		self.__route = route

	def PointCloud(self):
		route = self.__route
		for i in xrange(3):
			filename = route + str(100014 + i) + '.pcap'
			packets = PcaptoPoints(filename, 'Dual').Read_Pcap()
			udp = packets[0][0:900]
			gps = packets[1]
			
			pcl = RealTimePointCloud(udp,gps,'Dual')
			pts = pcl.PointCloud_init()
			print (str(len(pts['points'])) + ' points extracted from ' + str(100000 + i) + '.pcap')
			self.PedestrianCluster(pts['points'])


	def PedestrianCluster(self, ptss):
		# find the round
		azm_init = ptss[0][7]
		last_azm_index = 0
		for pn in xrange(len(ptss)-1):
			if ptss[pn][7]-azm_init<0 and ptss[pn+1][7]-azm_init>=0:
				opts = ptss[last_azm_index:pn+1]
				last_azm_index = pn+1
				print len(opts)

				# arrange the points with laser angle form bottom to top
				pts = np.empty([16*3600,10]).astype(np.int64)
				for i in xrange(len(opts)):
					if opts[i][3] % 2 == 0:
						pts[(opts[i][3] / 2)*3600+(i%3600)] = opts[i]
						pts[(opts[i][3] / 2)*3600+(i%3600)][1] = i	# replace block id with pid
					else:
						pts[((opts[i][3]+15)/2)*3600+(i%3600)] = opts[i]
						pts[((opts[i][3]+15)/2)*3600+(i%3600)][1] = i	# replace block id with pid


				# clean up points
				for i in xrange(16*3600):
					if pts[i][6] > 30000: 		# distance > 8m
						pts[i][6] = 0
						pts[i][8] = 0
						pts[i][9] = 0
					# elif pts[i][9] < -1500 or pts[i][9] > 1000: 	# z height < -5m or > 5m
					# 	pts[i][6] = 0
					# 	pts[i][8] = 0
					# 	pts[i][9] = 0


				# xyz analysis
				pts_x = np.empty([16,3600]).astype(np.int64)
				pts_y = np.empty([16,3600]).astype(np.int64)
				pts_z = np.empty([16,3600]).astype(np.int64)
				for i in xrange(16):
					for j in xrange(3600):
						n = 3600*i+j
						pts_x[i][j] = pts[n][6] * math.cos(pts[n][4]/360.0*(2*math.pi)) * math.sin(pts[n][7]/3600.0*(2*math.pi))
						pts_y[i][j] = pts[n][6] * math.cos(pts[n][4]/360.0*(2*math.pi)) * math.cos(pts[n][7]/3600.0*(2*math.pi))
						pts_z[i][j] = pts[n][6] * math.sin(pts[n][4]/360.0*(2*math.pi))
				pts_x = pts_x.reshape([16*3600,])
				pts_y = pts_y.reshape([16*3600,])
				pts_z = pts_z.reshape([16*3600,])
				pts_xyz = np.array([pts_x,pts_y,pts_z]).T
				pts_xyz = pts_xyz[~np.all(pts_xyz == 0, axis=1)]

				# rough cluster -- kd-tree
				DIM = 3
				from scipy.spatial import KDTree
				tree = KDTree(pts_xyz, leafsize=pts_xyz.shape[0]+1)
				# point = pts_xyz[99]
				# distances, ndx = tree.query([point], k=100)
				# print ndx
				# ndx = tree.query_ball_point(point, 150)
				# print ndx


		
				fig = plt.figure()
				pts_plot = fig.gca(projection = '3d')
				pts_plot.set_xlim3d(-15000,15000)
				pts_plot.set_ylim3d(-15000,15000)
				pts_plot.set_zlim3d(-15000,15000)
				


				# covariance matrix
				# pts_xyz_temp = np.copy(pts_xyz)
				# while np.any(pts_xyz_temp):
				# 	pt = pts_xyz_temp[0]
				# 	n1 = tree.query_ball_point(pt,150)
				# 	n2 = tree.query_ball_point(pt,200)
				# 	if n1 == n2:


				
				ped = []
				x = []
				y = []
				z = []
				ped_in_pts = False
				n = 0
				while not ped_in_pts and n<pts_xyz.shape[0]:
					pt = pts_xyz[n]



					n1 = tree.query_ball_point(pt,450)
					n2 = tree.query_ball_point(pt,600)




					if n1 == n2 and len(n1)>30:
						ped = pts_xyz[n1]
						ped_in_pts = True
					n += 1
				
				for ppt in ped:
					x += [ppt[0]]
					y += [ppt[1]]
					z += [ppt[2]]

				xo = []
				yo = []
				zo = []
				for pt in pts_xyz:
					xo += [pt[0]]
					yo += [pt[1]]
					zo += [pt[2]]

				print ped


				
				pts_plot.plot(xo,yo,zo,'g.')		
				pts_plot.plot(x,y,z,'r.')

				
				plt.show()







if __name__=='__main__':
	# PedestrianClass('/home/bowen/Desktop/lidar/pedestrian_full_scan/').PointCloud()

	ObjectDetection('/home/bowen/Desktop/lidar/pedestrian_full_scan/').PointCloud()

	# packets = PcaptoPoints('/home/bowen/Desktop/lidar/Cyber_Lab_Study_Sample/2016-09-20-13-12-25_Velodyne-VLP-16-Data-Dual.pcap', "Dual").Read_Pcap()
	# udp = packets[0]
	# gps = packets[1]
	
	# t1 = time.time()
	# pc= RealTimePointCloud(udp,gps,'Dual')
	# pts = pc.PointCloud_init()
	# t2 = time.time()
	# print t2-t1

	# print len(pts['points'])
