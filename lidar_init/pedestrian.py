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
	def __init__(self, UDP, GPS, MODE, UN, UNI):
		self.__udp = UDP 	# udp packets
		self.__gps = GPS 	# gps packets
		self.__mode = MODE
		self.__un = UN
		self.__uni = UNI
		self.__laser_angle = [-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]		# vertical angle according to laser ID

	def PointCloud_init(self):
		udps = self.__udp
		mode = self.__mode
		un = self.__un
		uni = self.__uni
		laser_angle = self.__laser_angle

		# initialize the point cloud environment with the first 'un' udp packets
		# register each point to the narray as 16(laser if) * 3600 (per 0.1 degree)
		round_check = False
		azimuth_init = False   
		pts = np.zeros([16*3600, 10], dtype = np.int64)
		ui = 0
		while not round_check and ui<un:
			u = ui+uni
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
			
			if azimuth_init:
				i = 0
				azimuth_ok = True
				while azimuth_ok and i<384:		
					ac = int(azimuth_dict[i/24] / 10)-1
					if abs(ac - start_azimuth) >= 5:
						lc = ptsc[i][3]
						depth = ptsc[i][6] * math.cos(math.radians(ptsc[i][4])) * math.sin(math.radians(ac/10.0))
						height = ptsc[i][6] * math.sin(math.radians(ptsc[i][4]))
						if depth != 0:
							pts[lc*3600+ac] = np.array(ptsc[i] + [ac, abs(depth), height])
						else:
							pts[lc*3600+ac] = np.array(ptsc[i] + [ac, 0, 0])
						i += 1
					else:
						azimuth_ok = False
						round_check = True
			else:
				for i in xrange(384):				
					ac = int(azimuth_dict[i/24] / 10)-1
					lc = ptsc[i][3]
					depth = ptsc[i][6] * math.cos(math.radians(ptsc[i][4])) * math.sin(math.radians(ac/10.0))
					height = ptsc[i][6] * math.sin(math.radians(ptsc[i][4]))
					if depth != 0:
						pts[lc*3600+ac] = np.array(ptsc[i] + [ac, abs(depth), height])
					else:
						pts[lc*3600+ac] = np.array(ptsc[i] + [ac, 0, 0])

				# record the first azimuth point with non-zero distance
				i = 0
				while i < 384 and not azimuth_init:
					if pts[i][6] != 0:
						azimuth_init = True
						start_azimuth = pts[i][7]
					i += 1
			ui += 1



		
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

		depth_pts = (pts.transpose()[[3,7,8]]).transpose()
		height_pts = (pts.transpose()[[3,7,9]]).transpose()

		Point_Cloud_Init = {	'points' : pts,
								'depth'  : depth_pts,
								'height' : height_pts }

		return Point_Cloud_Init

class PedestrianClass(object):

	def __init__(self, route):
		self.__route = route

	def PointCloud(self):
		route = self.__route
		for i in xrange(3):
			filename = route + str(100005 + i) + '.pcap'
			packets = PcaptoPoints(filename, 'Dual').Read_Pcap()
			udp = packets[0]
			gps = packets[1]
			for j in xrange(len(udp)/755):
				uni = j*755
				pcl = RealTimePointCloud(udp,gps,'Dual',755, uni)
				pts = pcl.PointCloud_init()
				print (str(pts['points'].shape[0]) + ' points extracted from ' + str(100000 + i) + '.pcap')
				self.PedestrianCluster(pts['points'])


	def PedestrianCluster(self, opts):
		# arrange the points with laser angle form bottom to top
		pts = np.empty([16*3600,10]).astype(np.int64)
		for i in xrange(16*3600):
			if opts[i][3] % 2 == 0:
				pts[(opts[i][3] / 2)*3600+(i%3600)] = opts[i]
				pts[(opts[i][3] / 2)*3600+(i%3600)][1] = i	# replace block id with pid
			else:
				pts[((pts[i][3]+15)/2)*3600+(i%3600)] = opts[i]
				pts[((pts[i][3]+15)/2)*3600+(i%3600)][1] = i	# replace block id with pid


		# clean up points
		for i in xrange(16*3600):
			if pts[i][6] > 3000: 		# distance > 8m
				pts[i][6] = 0
				pts[i][8] = 0
				pts[i][9] = 0
			elif pts[i][9] < -1500 or pts[i][9] > 1500: 	# z height < -5m or > 5m
				pts[i][6] = 0
				pts[i][8] = 0
				pts[i][9] = 0


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
			n1 = tree.query_ball_point(pt,400)
			n2 = tree.query_ball_point(pt,500)
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



		


		# 	if pt not in possip:

		# 		n1 = tree.query_ball_point(pt,150)
		# 		n2 = tree.query_ball_point(pt,200)
		# 		if n1 == n2 and len(n1)>30:
		# 			for n in n1:
		# 				possip += [pts_xyz[n1]]

		# for pt in possip:
		# 	x += pt[0]
		# 	y += pt[1]
		# 	z += pt[2]
		# pts_plot.plot(x,y,z,'.')		
		# plt.show()

		# x = np.unique(x_array)
		# y = np.unique(y_array)
		# z = np.unique(z_array)
			


		# if x.size > 0:
		# 	pts_plot.plot(x,y,z,'.')		
		# 	plt.show()
		# else:
		# 	pass
				


		# 	nearest_index = tree.query_ball_point(pt, 200)
		# 	nearest = pts_xyz[nearest_index]
		# 	cavr_x = []
		# 	cavr_y = []
		# 	cavr_z = []
		# 	for npt in nearest:
		# 		cavr_x += [npt[0] - pt[0]]
		# 		cavr_y += [npt[1] - pt[1]]
		# 		cavr_z += [npt[2] - pt[2]]
		# 	cavr = np.array([cavr_x, cavr_y, cavr_z])
		# 	cavr_matrix = np.dot(cavr, cavr.T)
		# 	cavr_eig = np.linalg.eigvals(cavr_matrix)
		# 	# print cavr_eig

		# 	if np.amin(cavr_eig) > 1000:
		# 		x_array += [pt[0]]
		# 		y_array += [pt[1]]
		# 		z_array += [pt[2]]
		# pts_plot.plot(x_array,y_array,z_array,'.')

		# plt.show()


			



		# pts = opts
		# fig = plt.figure()
		# pts_plot = fig.gca(projection = '3d')
		# pts_plot.set_xlim3d(-15000,15000)
		# pts_plot.set_ylim3d(-15000,15000)
		# pts_plot.set_zlim3d(-15000,15000)
		# x_array = []
		# y_array = []
		# z_array = []
		# for i in xrange(pts.shape[0]):
		# 	x_array += [pts[i][6] * math.cos(pts[i][4]/360.0*(2*math.pi)) * math.sin(pts[i][7]/3600.0*(2*math.pi))]
		# 	y_array += [pts[i][6] * math.cos(pts[i][4]/360.0*(2*math.pi)) * math.cos(pts[i][7]/3600.0*(2*math.pi))]
		# 	z_array += [pts[i][6] * math.sin(pts[i][4]/360.0*(2*math.pi))]
		# pts_plot.plot(x_array,y_array,z_array,'.')

		# plt.show()



		# 			cavr_x = []
		# 			cavr_y = []
		# 			cavr_z = []
		# 			for cx in [i-1, i, i+1]:
		# 				for cy in [j-1, j, j+1]:
		# 					if cx == i and cy == j:
		# 						pass
		# 					elif pts_height[cx][cy] != 0:
		# 						cavr_x += [pts_x[cx][cy]-pts_x[i][j]]
		# 						cavr_y += [pts_y[cx][cy]-pts_y[i][j]]
		# 						cavr_z += [pts_z[cx][cy]-pts_z[i][j]]
		# 					else:
		# 						cavr_x += [0]
		# 						cavr_y += [0]
		# 						cavr_z += [0]
		# 			cavr = np.array([cavr_x, cavr_y, cavr_z])
		# 			cavr_matrix = np.dot(cavr, cavr.T)
		# 			cavr_eig = np.linalg.eigvals(cavr_matrix)


		# 			covariance[i][j] = np.sum(cavr_eig)
		# 			print covariance[i][j]




		# print np.amax(distances)

		# convariance matrix







# 		#!/usr/bin/env python
# import numpy

# NDIM = 3 # number of dimensions

# # read points into array
# a = numpy.fromfile('million_3D_points.txt', sep=' ')
# a.shape = a.size / NDIM, NDIM

# point =  [ 69.06310224,   2.23409409,  50.41979143] # use the same point as above
# print 'point:', point


# from scipy.spatial import KDTree

# # find 10 nearest points
# tree = KDTree(a, leafsize=a.shape[0]+1)
# distances, ndx = tree.query([point], k=10)

# # print 10 nearest points to the chosen one
# print a[ndx]





		# pts_xyz_iter = list(range(2))

		# pts_xyz_iter[0] = np.delete(pts_xyz,[0],0)
		# pts_xyz_iter[1] = np.copy(pts_xyz[0])
		
		# reach_boundry = False
		# while not reach_boundry:




		# for pt in clustered[-1]:
		# 	for 
		
		# print pts_xyz_iter[1].shape
		# print pts_xyz_iter[0].shape








		# # covariance matrix
		# # 0: surface
		# covariance = np.empty([16,3600]).astype(np.float32)
		# for i in xrange(16):
		# 	for j in range(3600):
		# 		if i == 0 or i == 15 or j == 0 or j == 3599 or pts_height[i][j] == 0:
		# 			covariance[i][j] = 0
		# 		else:
		# 			cavr_x = []
		# 			cavr_y = []
		# 			cavr_z = []
		# 			for cx in [i-1, i, i+1]:
		# 				for cy in [j-1, j, j+1]:
		# 					if cx == i and cy == j:
		# 						pass
		# 					elif pts_height[cx][cy] != 0:
		# 						cavr_x += [pts_x[cx][cy]-pts_x[i][j]]
		# 						cavr_y += [pts_y[cx][cy]-pts_y[i][j]]
		# 						cavr_z += [pts_z[cx][cy]-pts_z[i][j]]
		# 					else:
		# 						cavr_x += [0]
		# 						cavr_y += [0]
		# 						cavr_z += [0]
		# 			cavr = np.array([cavr_x, cavr_y, cavr_z])
		# 			cavr_matrix = np.dot(cavr, cavr.T)
		# 			cavr_eig = np.linalg.eigvals(cavr_matrix)


		# 			covariance[i][j] = np.sum(cavr_eig)
		# 			print covariance[i][j]






	

		# scan the height img horizontally, keep the continuous height
		# for i in xrange(16):
		# 	lh = pts_height[i][0]
		# 	j = 1
		# 	while j < 3600:
		# 		if abs(pts_height[i][j] - lh) <= 100:
		# 			lh = pts_height[i][j]
		# 		else:
		# 			pts_height[i][j] = 0
		# 			pts[3600*i+j][6] = 0
		# 			pts[3600*i+j][8] = 0
		# 			pts[3600*i+j][9] = 0
		# 		j += 1

		

		# x_arr = []
		# y_arr = []
		# fig=plt.subplot(111)
		# fig.set_xlim([0, 15])
		# fig.set_ylim([0, 3604])
		# for i in xrange(16):
		# 	for j in xrange(3600):
		# 		if pts_height[i][j] != 0:
		# 			x_arr += [i]
		# 			y_arr += [j]
		# plt.plot(x_arr,y_arr,'.')
		# plt.show()



		# pts = np.delete(pts,np.where(pts.T[6]==0),axis=0)	# non-zero points
		# self.ShowMeThePoints(pts)






		# , replace firing sequence with laser order from 0 (-15) to 15 (+15)
		# for i in xrange(16*3600):
		# 	pts[i][1] = i
		# 	if pts[i][3] % 2 == 0:
		# 		pts[i][3] = pts[i][3] / 2
		# 	else:
		# 		pts[i][3] = (pts[i][3]+15)/2

		

		# # build 2d distance map
		# pts_dist = np.empty([16,3600]).astype(np.int64)
		# for i in xrange(16):
		# 	for j in xrange(3600):
		# 		pts_dist[i][j] = pts[3600*i+j][6]





		# # build (16*3600) 2D-DEPTH img
		# pts_depth = np.empty([16,3600]).astype(np.int64)
		# pts_height = np.empty([16,3600]).astype(np.int64)
		# for i in xrange(16):
		# 	for j in xrange(3600):
		# 		pts_depth[i][j] = pts[3600*i+j][8]
		# 		pts_height[i][j] = pts[3600*i+j][9]





		# # horizontal point cluster on depth img
		# hpc = list(range(16))
		# for i in xrange(16):
		# 	hpc[i] = [[]]
		# 	ldpt = 500000
		# 	for j in xrange(3600):
		# 		if pts_depth[i][j] == 0:
		# 			pass
		# 		else:
		# 			derr = abs(pts_depth[i][j]-lh)
		# 			if derr <= 100:
		# 				hpc[i][-1] += [j]
		# 			else:
		# 				hpc[i] += [[j]]
		# 				if len(hpc[i]) > 1 and len(hpc[i][-2]) == 1:
		# 					hpc[i].pop(-2)
		# 		lh = pts_depth[i][j]
		# 	if hpc[i][0] == []:
		# 		hpc[i].pop(0)
		# 	for k in xrange(len(hpc[i])):
		# 		hpc[i][k] = [i,max(hpc[i][k]),min(hpc[i][k])]


		# # point cluster on hpc (vertical)
		# clustered = hpc[0]
		# i = 1
		# while i < 16:
		# 	for n in xrange(len(clustered)):
		# 		cluster_count[n] = 0
		# 		for m in xrange(len(hpc[i])):
		# 			ci = clustered[n][0]
		# 			cmax = clustered[n][1]
		# 			cmin = clustered[n][2]
		# 			cmaxd = pts_depth[ci][cmax]
		# 			cmind = pts_depth[ci][cmin]
		# 			hi = hpc[i][m][0]
		# 			hmax = hpc[i][m][1]
		# 			hmin = hpc[i][m][2]
		# 			hmaxd = pts_depth[hi][hmax]
		# 			hmind = pts_depth[hi][hmin]
		# 			if cmax > hmin and cmin < hmax and abs(cmaxd-hmaxd)<=100 and abs(cmind-hmind)<=100:
		# 				clustered[n] += hpc[i][m]




		# 			if clustered[n][1] >= hpc[i][m][2] and clustered[n][2] <= hpc[i][m][1] and 
		# 				clustered[n] += hpc[i][m]
		# 				cluster_count[n] += 1
		# 				hpc[i][m] = []
					



				


		# print hpc[5]



		

		


		# from scipy import ndimage 

		# def find_clusters(array):
		#     clustered = np.empty_like(array)
		#     unique_vals = np.unique(array)
		#     cluster_count = 0
		#     for val in unique_vals:
		#         labelling, label_count = ndimage.label(array == val)
		#         for k in range(1, label_count + 1):
		#             clustered[labelling == k] = cluster_count
		#             cluster_count += 1
		#     return clustered, cluster_count

		# clusters, cluster_count = find_clusters(array)
		# print("Found {} clusters:".format(cluster_count))
		# print(clusters)

		# ones = np.ones_like(array, dtype=int)
		# cluster_sizes = ndimage.sum(ones, labels=clusters, index=range(cluster_count)).astype(int)
		# com = ndimage.center_of_mass(ones, labels=clusters, index=range(cluster_count))
		# for i, (size, center) in enumerate(zip(cluster_sizes, com)):
		#     print("Cluster #{}: {} elements at {}".format(i, size, center))




		# print pts_depth





		# close points cluster (kd tree)



		# temp_pts = np.copy(pts)


		# while temp_pts:





		# i = 0
		# j = 0
		# while i < 16:
		# 	while j < 3600:
		# 		if temp_pts[i*3600+j][6] == 0:
		# 			i += 1



		# 		while temp_pts[i*3600+j][6] != 0:
		# 			i += 1









		# principle compoment analysis


		# point feature histogram



		# print pts




		# pts = np.delete(pts,np.where(pts.T[6]==0),axis=0)	# non-zero points


		# temp_pts = np.copy(pts)
		

		




		# # column fit 
		# for i in xrange(pts.shape[0]):
		# 	x = pts[i][6] * math.cos(pts[i][4]/360.0*(2*math.pi)) * math.sin(pts[i][7]/3600.0*(2*math.pi))
		# 	y = pts[i][6] * math.cos(pts[i][4]/360.0*(2*math.pi)) * math.cos(pts[i][7]/3600.0*(2*math.pi))
		# 	z = pts[i][6] * math.sin(pts[i][4]/360.0*(2*math.pi))
			
			


		# print pts.shape
		# print pts
		




		# self.ShowMeThePoints(obj)
		


# np.where(a.T[3]==0)


		# delete all zeros
		

		


		# print obj.shape
		# print obj
		# self.ShowMeThePoints(obj)
				


	def ShowMeThePoints(self, opts):
		pts = opts
		fig = plt.figure()
		pts_plot = fig.gca(projection = '3d')
		pts_plot.set_xlim3d(-15000,15000)
		pts_plot.set_ylim3d(-15000,15000)
		pts_plot.set_zlim3d(-15000,15000)
		x_array = []
		y_array = []
		z_array = []
		for i in xrange(pts.shape[0]):
			x_array += [pts[i][6] * math.cos(pts[i][4]/360.0*(2*math.pi)) * math.sin(pts[i][7]/3600.0*(2*math.pi))]
			y_array += [pts[i][6] * math.cos(pts[i][4]/360.0*(2*math.pi)) * math.cos(pts[i][7]/3600.0*(2*math.pi))]
			z_array += [pts[i][6] * math.sin(pts[i][4]/360.0*(2*math.pi))]
		pts_plot.plot(x_array,y_array,z_array,'.')

		plt.show()
		


			


if __name__=='__main__':
	PedestrianClass('/home/bowen/Desktop/lidar/pedestrian_full_scan/').PointCloud()




	# packets = PcaptoPoints('/home/bowen/Desktop/lidar/Cyber_Lab_Study_Sample/2016-09-20-13-12-25_Velodyne-VLP-16-Data-Dual.pcap', "Dual").Read_Pcap()
	# udp = packets[0]
	# gps = packets[1]
	
	# t1 = time.time()
	# pc= RealTimePointCloud(udp,gps,'Dual')
	# pts = pc.PointCloud_init()
	# t2 = time.time()
	# print t2-t1

	
	# # print pts
	# origin_pts = pts['points']
	# depth_pts = pts['depth']
	# height_pts = pts['height']
	# # print np.amax(height_pts.transpose()[[2]])
	# # print np.amin(height_pts.transpose()[[2]])
	# n = np.zeros(16)
	# m = np.zeros(3600)
	# for i in xrange(16*3600):
	# 	if depth_pts[i][2] == 0:
	# 		n[depth_pts[i][0]] += 1
	# 		m[depth_pts[i][1]] += 1

	# print m
	# print n