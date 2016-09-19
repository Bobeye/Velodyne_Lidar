import numpy as np
import matplotlib.pyplot as plt

class Plane_Segmentation(object):
	def __init__(self, plane_segmentation_point_cloud):
		self.__point_cloud = plane_segmentation_point_cloud


	def Ground_Classification(self):	# distinguish points from ground and nonground
		obvious_gound_threshold = -3000
		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)
		
		# extract possible ground
		not_ground = -1000
		possible_ground_cloud = [0]
		for i in range(points_num):
			if init_point_cloud[i][5] < not_ground:
				possible_ground_cloud.append(init_point_cloud[i])
		possible_ground_cloud.pop(0)

		# cluster closed points based on Aziumth and laser id



	def Build_Point_Voxel(self):
		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)
		voxel_res = 1000 # mm
		# find the range of x, y, z
		x_range = [0,0]
		y_range = [0,0]
		z_range = [0,0]
		for i in xrange(points_num):
			x = init_point_cloud[i][3]
			y = init_point_cloud[i][4]
			z = init_point_cloud[i][5]
			if x_range[0] > x:
				x_range[0] = x
			if x_range[1] < x:
				x_range[1] = x
			if y_range[0] > y:
				y_range[0] = y
			if y_range[1] < y:
				y_range[1] = y
			if z_range[0] > z:
				z_range[0] = z
			if z_range[1] < z:
				z_range[1] = z
		# cluster the cloud into voxel
		point_locater = [[0],[0]]
		voxel_list = [0]
		for i in xrange(points_num):
			x = init_point_cloud[i][3] - x_range[0]
			y = init_point_cloud[i][4] - y_range[0]
			z = init_point_cloud[i][5] - z_range[0]
			xyz_id = [int(x)/voxel_res, int(y)/voxel_res, int(z)/voxel_res]
			identifier = 0
			for j in xrange(len(point_locater[0])):
				if xyz_id == point_locater[0][j]:
					voxel_list[point_locater[1][j]].append(init_point_cloud[i])
					identifier = 1
			if identifier == 0:
				point_locater[0].append(xyz_id)
				point_locater[1].append(len(point_locater[0])-1)
				voxel_list.append([init_point_cloud[i]])
		voxel_list.pop(0)
		
		return voxel_list





		# print len(voxel_list)
		# n = 0
		# b = 0
		# for i in xrange(len(voxel_list)):
		# 	if len(voxel_list[i]) == 1:
		# 		n += 1
		# 	if len(voxel_list[i]) > b:
		# 		b = len(voxel_list[i])
		# print n
		# print b

		# print voxel_list[78]


		# 	if xyz_id not in point_locater[0]:
		# 		point_locater[0].append(xyz_id)
		# 		point_locater[1].append(len(point_locater[0]))
		# 		voxel_list
		# 	else:









		# temp_voxel = list(range((int(x_range[1]-x_range[0]) / voxel_res) + 1))
		# for i in xrange(len(temp_voxel)):
		# 	temp_voxel[i] = [0]
		# for i in xrange(points_num):
		# 	x = init_point_cloud[i][3] - x_range[0]
		# 	temp_voxel[int(x)/voxel_res].append(init_point_cloud[i])
		# x_temp_voxel = [0]
		# for i in xrange(len(temp_voxel)):
		# 	if temp_voxel[i] != [0]:
		# 		temp_voxel[i].pop(0)
		# 		x_temp_voxel.append(temp_voxel[i])
		# x_temp_voxel.pop(0)
		# xy_temp_voxel = list(range(x_temp_voxel))

		# for i in range(len(xy_temp_voxel)):
		# 	xy_temp_voxel[i] = list(range((int(y_range[1]-y_range[0]) / voxel_res) + 1))
		# 	for j in range(len(xy_temp_voxel[i])):
		# 		xy_temp_voxel[i][j] = [0]
		# for i in xrange(len(x_temp_voxel)):
		# 	for j in range(len(x_temp_voxel[i])):
		# 		x_temp_voxel[i][j][4]
		# 		xy_temp_voxel[i][j].append(x_temp_voxel[i])





		# Voxel_size = ((int(x_range[1]-x_range[0]) / voxel_res) + 1) \
		# 		   * ((int(y_range[1]-y_range[0]) / voxel_res) + 1) \
		# 		   * ((int(z_range[1]-z_range[0]) / voxel_res) + 1)
		# print Voxel_size
		# temp_voxel = list(range(Voxel_size))
		


		# for i in xrange(points_num):
		# 	temp_voxel[i] = [0]
		# for i in xrange(points_num):
		# 	x = init_point_cloud[i][3] - x_range[0]
		# 	y = init_point_cloud[i][4] - y_range[0]
		# 	z = init_point_cloud[i][5] - z_range[0]
		# 	v_x = int(x) / voxel_res
		# 	v_y = int(y) / voxel_res
		# 	v_z = int(z) / voxel_res
		# 	temp_voxel[v_x + v_y + v_z].append = init_point_cloud[i]
		# voxel_list = [0]
		# for i in xrange(Voxel_size):
		# 	if temp_voxel[Voxel_size] != [0]:
		# 		voxel_list.append(temp_voxel[Voxel_size])
		# voxel_list.pop(0)

		# print len(voxel_list)








	def Build_Point_Grid(self):
		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)


	def Build_Point_Cloud_Array(self):
		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)
		# build pck format of point cloud: points
		points_list = [[init_point_cloud[0][3],init_point_cloud[0][4],init_point_cloud[0][5]]]
		for i in range(points_num-1):
			points_list.append([init_point_cloud[i+1][3],init_point_cloud[i+1][4],init_point_cloud[i+1][5]])
		points_array = np.array(points_list,dtype=np.float32)

		return points_array

	def Build_Point_Cloud(self):
		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)
		# build pck format of point cloud: points
		points_list = [[init_point_cloud[0][3],init_point_cloud[0][4],init_point_cloud[0][5]]]
		for i in range(points_num-1):
			points_list.append([init_point_cloud[i+1][3],init_point_cloud[i+1][4],init_point_cloud[i+1][5]])
		
		return points_list

	def Fit_Plane_SVD(self):	# slower than LTSQ
		cloud = self.Build_Point_Cloud()
		[rows, cols] = cloud.shape
		# Set up constraint equations of the form  AB = 0,
    	# where B is a column vector of the plane coefficients
    	# in the form b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0.
		p = (np.ones((rows,1)))
		AB = np.hstack([cloud,p])
		[u, d, v] = np.linalg.svd(AB,0)        
		B = v[3,:];                    # Solution is last column of v.
		nn = np.linalg.norm(B[0:3])
		B = B / nn
		return B[0:3]

	def Fit_Plane_LTSQ(self):
		cloud = self.Build_Point_Cloud()
		# Fits a plane to a point cloud, 
		# Where Z = aX + bY + c
		# Rearanging Eqn1: aX + bY -Z +c =0
		# Gives normal (a,b,-1)
		# Normal = (a,b,-1)
		[rows,cols] = cloud.shape
		G = np.ones((rows,3))
		G[:,0] = cloud[:,0]  #X
		G[:,1] = cloud[:,1]  #Y
		Z = cloud[:,2]
		(a,b,c),resid,rank,s = np.linalg.lstsq(G,Z) 
		normal = (a,b,-1)
		nn = np.linalg.norm(normal)
		normal = normal / nn
		return normal
