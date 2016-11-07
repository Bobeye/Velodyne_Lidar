import numpy as np
import numbapro
from numbapro import guvectorize, vectorize, cuda
import time
import math


# [-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]


"""
In: [original point cloud, number of points, recursive point cloud, number of recursive points,[classification1,2,3...]]
"""
class PointCloud(object):

	def __init__(self, pco, npco, pcr, npcr, pcc):
		self.__pco = pco 	# original point cloud
		self.__npco = npco 	# number of pco points
		self.__pcr = pcr 	# recursive point cloud
		self.__npcr = npcr 	# number of recursive points
		self.__pcc = pcc 	# a list of pco classifications

		# special specs
		self.__yres_yzc = 500		# 1mm per res, y resolution for yz-classification
		self.__zres_yzc = 50		# 1mm per res, z resolution for yz-classification
		self.__ymax_yzc = 50000	# mm
		self.__ymin_yzc = -50000
		self.__zmax_yzc = 10000		# mm
		self.__zmin_yzc = -10000 	# mm	

	def velodyne_lidar_pc_classfication(self):
		# get the list of original points
		"""
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

		laser ID's angle info: [-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]
		"""
		pco = self.__pco
		npco = self.__npco

		# yz classfication
		"""
		yz-class:
		[ 00 yz-class-id,
		  01 x id (1 for x>=0, 0 for x<0), 
		  02 number of points,
		  03 depth mean (x),
		  04 max depth,
		  05 min depth,
		  06 y mean,
		  07 max y,
		  08 min y,
		  09 z mean,
		  10 max z,
		  11 min z,
		  12 reflectivity mean,
		  13 reflectivity max,
		  14 reflectivity min]
		yz-id:
		[ point id, yz-class-id ]
		"""
		y_res_yzc = self.__yres_yzc
		z_res_yzc = self.__zres_yzc
		y_max_yzc = self.__ymax_yzc
		y_min_yzc = self.__ymin_yzc
		z_max_yzc = self.__zmax_yzc
		z_min_yzc = self.__zmin_yzc
		y_rng_yzc = y_max_yzc - y_min_yzc
		z_rng_yzc = z_max_yzc - z_min_yzc
		pid_yz_class = list(range(npco))
		yz_class_init = np.zeros([(y_rng_yzc/y_res_yzc+1)*(z_rng_yzc/z_res_yzc+1)*2, 15], dtype = np.int64)
		for pid in xrange(npco):
			lid, xo, yo, zo, ro, do, ao, to = pco[pid][2],pco[pid][3],pco[pid][4],pco[pid][5],pco[pid][6],pco[pid][7],pco[pid][8],pco[pid][9]
			# 00
			yz_class_id = ((int(yo-y_min_yzc)/y_res_yzc)+1) * ((int(zo-z_min_yzc)/z_res_yzc)+1)	# y id * (y range / y res) + z id
			# 01
			xid = (xo>=0)*1 + (xo<0)*0
			if xid == 1:
				yz_class_id = yz_class_id + (y_rng_yzc/y_res_yzc+1)*(z_rng_yzc/z_res_yzc+1)
			else:
				pass
			if yo > y_max_yzc or yo < y_min_yzc or zo > z_max_yzc or zo < z_min_yzc or xo == 0:
				# for i in xrange(15):
				# 	yz_class_init[yz_class_id][i] = 0
				pass
			else:
				if yz_class_init[yz_class_id][2] == 0:
					npts = 1	# 02
					x_mean = abs(xo)
					x_max = abs(xo)
					x_min = abs(xo)
					y_mean = yo
					y_max = yo
					y_min = yo
					z_mean = zo
					z_max = zo
					z_min = zo
					r_mean = ro
					r_max = ro
					r_min = ro
					yz_class_init[yz_class_id][0] = int(yz_class_id)
					yz_class_init[yz_class_id][1] = xid
					yz_class_init[yz_class_id][2] = npts
					yz_class_init[yz_class_id][3] = int(x_mean)
					yz_class_init[yz_class_id][4] = int(x_max)
					yz_class_init[yz_class_id][5] = int(x_min)
					yz_class_init[yz_class_id][6] = int(y_mean)
					yz_class_init[yz_class_id][7] = int(y_max)
					yz_class_init[yz_class_id][8] = int(y_min)
					yz_class_init[yz_class_id][9] = int(z_mean)
					yz_class_init[yz_class_id][10] = int(z_max)
					yz_class_init[yz_class_id][11] = int(z_min)
					yz_class_init[yz_class_id][12] = int(r_mean)
					yz_class_init[yz_class_id][13] = int(r_max)
					yz_class_init[yz_class_id][14] = int(r_min)
				elif yz_class_init[yz_class_id][2] != 0:
					yz_class_init[yz_class_id][0] = int(yz_class_id)
					yz_class_init[yz_class_id][1] = xid
					yz_class_init[yz_class_id][2] = yz_class_init[yz_class_id][2] + 1
					xo = abs(xo)
					yz_class_init[yz_class_id][3] = ((yz_class_init[yz_class_id][3] * (yz_class_init[yz_class_id][2]-1)) + xo) / yz_class_init[yz_class_id][2]
					yz_class_init[yz_class_id][4] = (yz_class_init[yz_class_id][4]<xo)*xo + (yz_class_init[yz_class_id][4]>=xo)*yz_class_init[yz_class_id][4]
					yz_class_init[yz_class_id][5] = (yz_class_init[yz_class_id][5]>xo)*xo + (yz_class_init[yz_class_id][5]<=xo)*yz_class_init[yz_class_id][5]
					yz_class_init[yz_class_id][6] = ((yz_class_init[yz_class_id][6] * (yz_class_init[yz_class_id][2]-1)) + yo) / yz_class_init[yz_class_id][2]
					yz_class_init[yz_class_id][7] = (yz_class_init[yz_class_id][7]<yo)*yo + (yz_class_init[yz_class_id][7]>=yo)*yz_class_init[yz_class_id][7]
					yz_class_init[yz_class_id][8] = (yz_class_init[yz_class_id][8]>yo)*yo + (yz_class_init[yz_class_id][8]<=yo)*yz_class_init[yz_class_id][8]
					yz_class_init[yz_class_id][9] = ((yz_class_init[yz_class_id][9] * (yz_class_init[yz_class_id][2]-1)) + zo) / yz_class_init[yz_class_id][2]
					yz_class_init[yz_class_id][10] = (yz_class_init[yz_class_id][10]<zo)*zo + (yz_class_init[yz_class_id][10]>=xo)*yz_class_init[yz_class_id][10]
					yz_class_init[yz_class_id][11] = (yz_class_init[yz_class_id][11]>zo)*zo + (yz_class_init[yz_class_id][11]<=xo)*yz_class_init[yz_class_id][11]
					yz_class_init[yz_class_id][12] = ((yz_class_init[yz_class_id][12] * (yz_class_init[yz_class_id][2]-1)) + ro) / yz_class_init[yz_class_id][2]
					yz_class_init[yz_class_id][13] = (yz_class_init[yz_class_id][13]<ro)*ro + (yz_class_init[yz_class_id][13]>=ro)*yz_class_init[yz_class_id][13]
					yz_class_init[yz_class_id][14] = (yz_class_init[yz_class_id][14]>ro)*ro + (yz_class_init[yz_class_id][14]<=ro)*yz_class_init[yz_class_id][14]
				
				
				# record the yz class id flag
				pid_yz_class[pid] = [pid, yz_class_id]


		point_cloud_classification = {  'point cloud' : pco , 
										'yz class' : [yz_class_init, pid_yz_class]}		

		return point_cloud_classification

	def feature_detection_yz_class(self):
		############################################################################
		# Controls whether to manually handle CUDA memory allocation or not.
		MANAGE_CUDA_MEMORY = True

		@guvectorize(['void(int32[:], int32[:])','void(int64[:], int64[:])'], '(n)->(n)', target='cuda')
		def yzc_pts_tag(p_in, r_out):
			# unpack parameters
			yzc_id = p_in[0]
			xid = p_in[1]
			npts = p_in[2]
			xmean = p_in[3]
			xmax = p_in[4]
			xmin = p_in[5]
			ymean = p_in[6]
			ymax = p_in[7]
			ymin = p_in[8]
			zmean = p_in[9]
			zmax = p_in[10]
			zmin = p_in[11]
			rmean = p_in[12]
			rmax = p_in[13]
			rmin = p_in[14]

			gw = 1

			if zmax >= 10:
				gw = gw * 0.69
			if zmin >= 10:
				gw = gw * 0.79
			if zmean >= -500:
				gw = gw * 0.79
			if npts <= 35:
				gw = gw * 0.87
			# if xmean*1.1<=xmax and xmean*0.9>=xmin:
			# 	gw = gw * 0.77
			if xmean < 1000 or xmean > 12000:
				gw = gw * 0.69

			if gw > 0.7:
				r_out[0] = 1
			else:
				r_out[0] = 0
		#########################################################################



		yz_class = self.velodyne_lidar_pc_classfication()['yz class']
		yz_class_init = yz_class[0]
		yz_class_pid = yz_class[1]		
		if MANAGE_CUDA_MEMORY:
		    # invoke on CUDA with manually managed memory
		    out = np.empty_like(yz_class_init)
		    dev_inp = cuda.to_device(yz_class_init)             # alloc and copy input data
		    dev_out = cuda.to_device(out, copy=False) # alloc only
		    yzc_pts_tag(dev_inp, out=dev_out)             # invoke the function
		    dev_out.copy_to_host(out)                 # retrieve the result
		else:
		    # Manually managing the CUDA allocation is optional, but recommended
		    # for maximum performance.
		    out = yzc_pts_tag(yz_class_init)

		# take the first 5 arrays
		gw = out.transpose()[:1]

		# for g in xrange(gw.shape[1]):
		# 	if gw[0][i] == 1


		return gw






if __name__=='__main__':

	import pcap
	# Read_Pcap('/home/poopeye/Desktop/WorkRelated/lidar/VELODYNE/VLP-16 Sample Data/2015-07-23-14-37-22_Velodyne-VLP-16-Data_Downtown 10Hz Single.pcap', "Single")

	# Read_Pcap('/home/poopeye/Desktop/WorkRelated/02-Marrysville-2016-09-13-14-18-15_Velodyne-VLP-16-Data.pcap', "Dual")

	mypcap = pcap.READPCAP('/home/bowen/Desktop/lidar/MovingVehicleRecord/2016-09-28-14-35-03_Velodyne-VLP-16-Data-Side.pcap', "Single")
	pts = mypcap.PCAP()

	npts = len(pts)

	# for pt in pts:
	# 	if pt[3] > 0:
	# 		print pt

	# print pts[:100]

	my_point_cloud = PointCloud(pts, npts, 0,0,0)
	yz_class_id = my_point_cloud.velodyne_lidar_pc_classfication()['yz class'][1]
	# print yz_class.shape
	# for yzc in yz_class:
	# 	# if yzc[1] == 1:
	# 	# 	print yzc
	# 	if np.any(yzc):
	# 		print yzc[10]


	gw = my_point_cloud.feature_detection_yz_class()
	for i in xrange(len(yz_class_id)):
		if type(yz_class_id[i]).__name__ == 'list':
			if gw[0][yz_class_id[i][1]] == 1:
				print pts[i]




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
	test_pcl_plot.set_xlim3d(-20000, 20000)
	test_pcl_plot.set_ylim3d(-20000, 20000)
	test_pcl_plot.set_zlim3d(-2000, 2000)
	x_array = []
	y_array = []
	z_array = []
	for i in xrange(len(yz_class_id)):
		if type(yz_class_id[i]).__name__ == 'list':
			if pts[i][7] > 12000:
				pass
			elif gw[0][yz_class_id[i][1]] == 1 and pts[i][5]< -1500:
				x_array += [pts[i][3]]
				y_array += [pts[i][4]]
				z_array += [pts[i][5]]
	test_pcl_plot.plot(x_array,y_array,z_array,'.')
	plt.show()


















# class PointCloud(object):

# 	def __init__(self, nest_pts, id_pts):
# 		self.__pts = nest_pts
# 		self.__id_pts = id_pts

# 	def point_tag(self):
# 		############################################################################
# 		# Controls whether to manually handle CUDA memory allocation or not.
# 		MANAGE_CUDA_MEMORY = True

# 		@guvectorize(['void(float32[:], float32[:])','void(float64[:], float64[:])'], '(n)->(n)', target='cuda')
# 		def pts_tag(p_in, r_out):
# 			# unpack parameters
# 			n, x, y, z, r, d, a, lid = p_in[0],p_in[1],p_in[2],p_in[3],p_in[4],p_in[5],p_in[6],p_in[7]
# 			gw = 1	# ground
# 			vw = 1	# vehicle
# 			nw = 1	# nothing

			

# 			##################	distance
# 			if d <= 1000 or d >= 18000:
# 				gw = gw * 0.57
# 				vw = vw * 0.57
# 				nw = nw * 0.99

# 			##################	z height
# 			if z < -1500:
# 				gw = gw * 0.99
# 				vw = vw * 0.68
# 				nw = nw * 0.78
# 			elif z >= -1500 and z <= 5000:
# 				gw = gw * 0.78
# 				vw = vw * 0.99
# 				nw = nw * 0.85
# 			else:
# 				gw = gw * 0.58
# 				vw = vw * 0.78
# 				nw = nw * 0.9

# 			#################### lane vote (x)
# 			if x <= 1 and x >= -1:
# 				gw = gw * 0.57
# 				vw = vw * 0.57
# 				nw = nw * 0.99

# 			elif x <= 8 and x >= -10:
# 				gw = gw * 0.8
# 				vw = vw * 0.99
# 				nw = nw * 0.57
# 			else:
# 				pass


# 			# ################## reflectivity
# 			# if r > 45:
# 			# 	nw = nw * 0.99
# 			# else:
# 			# 	nw = nw * 0.57


# 			# ################## lid
# 			# lid = int(lid)
# 			# gw = gw * ((lid == 0)*0.99 \
# 			# 		 + (lid == 2)*0.98 \
# 			# 		 + (lid == 4)*0.96 \
# 			# 		 + (lid == 6)*0.94 \
# 			# 		 + (lid == 8)*0.92 \
# 			# 		 + (lid == 10)*0.89 \
# 			# 		 + (lid == 12)*0.79 \
# 			# 		 + (lid == 14)*0.7 \
# 			# 		 + (lid == 1)*0.59 \
# 			# 		 + (lid == 3)*0.587 \
# 			# 		 + (lid == 5)*0.583 \
# 			# 		 + (lid == 7)*0.579 \
# 			# 		 + (lid == 9)*0.574 \
# 			# 		 + (lid == 11)*0.573 \
# 			# 		 + (lid == 13)*0.571 \
# 			# 		 + (lid == 15)*0.57)
# 			# vw = vw * ((lid == 0)*0.72 \
# 			# 		 + (lid == 2)*0.81 \
# 			# 		 + (lid == 4)*0.9 \
# 			# 		 + (lid == 6)*0.98 \
# 			# 		 + (lid == 8)*0.945 \
# 			# 		 + (lid == 10)*0.923 \
# 			# 		 + (lid == 12)*0.888 \
# 			# 		 + (lid == 14)*0.852 \
# 			# 		 + (lid == 1)*0.789 \
# 			# 		 + (lid == 3)*0.667 \
# 			# 		 + (lid == 5)*0.613 \
# 			# 		 + (lid == 7)*0.589 \
# 			# 		 + (lid == 9)*0.574 \
# 			# 		 + (lid == 11)*0.573 \
# 			# 		 + (lid == 13)*0.571 \
# 			# 		 + (lid == 15)*0.57)




# 			if gw > vw and gw > nw:
# 				r_out[0] = 0
# 			elif vw > gw and vw > nw:
# 				r_out[0] = 1
# 			else:
# 				r_out[0] = 2
# 		#########################################################################

# 		inp = self.__pts
# 		id_pts = self.__id_pts
	
# 		if MANAGE_CUDA_MEMORY:
# 		    # invoke on CUDA with manually managed memory
# 		    out = np.empty_like(inp)
# 		    dev_inp = cuda.to_device(inp)             # alloc and copy input data
# 		    dev_out = cuda.to_device(out, copy=False) # alloc only
# 		    pts_tag(dev_inp, out=dev_out)             # invoke the function
# 		    dev_out.copy_to_host(out)                 # retrieve the result
# 		else:
# 		    # Manually managing the CUDA allocation is optional, but recommended
# 		    # for maximum performance.
# 		    out = pts_tag(inp)

# 		# take the first 5 arrays
# 		out = out.transpose()[:1]


# 		# for i in xrange(out.shape[1]):
# 		# 	if out[0][i] == 1:
# 		# 		print inp[i]	

# 		return out