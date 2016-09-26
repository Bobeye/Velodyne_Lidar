import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

class Plane_Segmentation(object):
	def __init__(self, plane_segmentation_point_cloud):
		self.__point_cloud = plane_segmentation_point_cloud


	def Ground_Classification(self):	# distinguish points from ground and nonground
		def Fit_Plane_LTSQ(point_cloud):
			cloud = point_cloud
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

		def Fit_Plane_SVD(point_cloud):	# slower than LTSQ
			cloud = point_cloud
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



		def points_deviation_mean(data_points):
			points = data_points

			x_sum = 0.0
			y_sum = 0.0
			z_sum = 0.0
			for i in xrange(len(points)):
				x_sum = x_sum + points[i][3]
				y_sum = y_sum + points[i][4]
				z_sum = z_sum + points[i][5]
			x_mean = x_sum / len(points)
			y_mean = y_sum / len(points)
			z_mean = z_sum / len(points)
			x_var = 0.0
			y_var = 0.0
			z_var = 0.0
			for i in xrange(len(points)):
				x_var = x_var + ((points[i][3] - x_mean) ** 2)
				y_var = y_var + ((points[i][4] - y_mean) ** 2)
				z_var = z_var + ((points[i][5] - z_mean) ** 2)
			x_var = x_var / len(points)
			y_var = y_var / len(points)
			z_var = z_var / len(points)
			x_dev = x_var ** 0.5
			y_dev = y_var ** 0.5
			z_dev = z_var ** 0.5

			return [x_dev,y_dev,z_dev,x_mean,y_mean,z_mean]

		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)

		# build ground column
		columb_res = 1000
		# range x & y
		x_range = [0,0]
		y_range = [0,0]
		for i in xrange(points_num):
			x = init_point_cloud[i][3]
			y = init_point_cloud[i][4]
			if x_range[0] > x:
				x_range[0] = x
			if x_range[1] < x:
				x_range[1] = x
			if y_range[0] > y:
				y_range[0] = y
			if y_range[1] < y:
				y_range[1] = y
		column_list = [0]
		point_locater = [[0],[0]]
		for i in xrange(points_num):
			x = init_point_cloud[i][3] - x_range[0]
			y = init_point_cloud[i][4] - y_range[0]
			xy_id = [int(x)/columb_res, int(y)/columb_res]
			identifier = 0
			for j in xrange(len(point_locater[0])):
				if xy_id == point_locater[0][j]:
					column_list[point_locater[1][j]].append(init_point_cloud[i])
					identifier = 1
			if identifier == 0:
				point_locater[0].append(xy_id)
				point_locater[1].append(len(point_locater[0])-1)
				column_list.append([init_point_cloud[i]])
		column_list.pop(0)
		# arange the points in each column
		for i in xrange(len(column_list)):
			if len(column_list[i]) > 1:
				temp_column = [column_list[i][0]]
				for j in xrange(len(column_list[i])-1):
					if column_list[i][j+1] >= temp_column[len(temp_column)-1]:
						temp_column.append(column_list[i][j+1])
					else:
						temp_column.insert(0,column_list[i][j+1])
				column_list[i] = temp_column
		# slice the column
		slice_res = 50
		for i in xrange(len(column_list)):
			if len(column_list[i]) > 1:
				temp_column = [[column_list[i][0]]]
				for j in xrange(len(column_list[i])-1):
					if column_list[i][j+1][5] - temp_column[len(temp_column)-1][len(temp_column[len(temp_column)-1])-1][5] >= slice_res:
						temp_column.append([column_list[i][j+1]])
					else:
						temp_column[len(temp_column)-1].append(column_list[i][j+1])
				column_list[i] = temp_column
			else:
				column_list[i] = [column_list[i]]

		# possible ground
		possible_ground = [0]
		possible_ground_signature = [0]
		possible_ground_threshold = 15 	# the minimum number of points that's inside a single voxel
		for i in xrange(len(column_list)):
			if len(column_list[i]) == 1 and len(column_list[i][0]) > possible_ground_threshold:
				points_dev_m = points_deviation_mean(column_list[i][0])
				possible_ground_signature.append(points_dev_m)
				possible_ground.append(column_list[i][0])
		possible_ground.pop(0)
		possible_ground_signature.pop(0)

		print("number of possible_ground voxels: ", len(possible_ground))

		# arrange x_mean and y_mean from min to max for further searching
		x_mean_list = [possible_ground_signature[0][3]]
		y_mean_list = [possible_ground_signature[0][4]]
		for i in range(len(possible_ground_signature)):
			x_mean = possible_ground_signature[i][3]
			y_mean = possible_ground_signature[i][4]
			if x_mean < x_mean_list[0]:
				x_mean_list.insert(0,x_mean)
			else:
				x_mean_list.append(x_mean)
			if y_mean < y_mean_list[0]:
				y_mean_list.insert(0,y_mean)
			else:
				y_mean_list.append(y_mean)


		# flat & non-flat
		possible_ground_flat = [0]
		possible_ground_nonflat = [0]
		possible_ground_flat_sig = [0]
		possible_ground_nonflat_sig = [0]
		for i in xrange(len(possible_ground)):
			x_dev = possible_ground_signature[i][0]
			y_dev = possible_ground_signature[i][1]
			z_dev = possible_ground_signature[i][2]
			if z_dev <= 20:
				possible_ground_flat.append(possible_ground[i])
				possible_ground_flat_sig.append(possible_ground_signature[i])
			else:
				possible_ground_nonflat.append(possible_ground[i])
				possible_ground_nonflat_sig.append(possible_ground_signature[i])
		possible_ground_flat.pop(0)
		possible_ground_flat_sig.pop(0)
		possible_ground_nonflat.pop(0)
		possible_ground_nonflat_sig.pop(0)


		# iluminent the deviatives
		z_mean_mean = 0
		z_mean_dev = 0
		for i in range(len(possible_ground_flat_sig)):
			z_mean_mean = z_mean_mean + possible_ground_flat_sig[i][5]
		z_mean_mean = z_mean_mean / len(possible_ground_flat_sig)
		for i in range(len(possible_ground_flat_sig)):
			z_mean_dev = z_mean_dev + (possible_ground_flat_sig[i][5] - z_mean_mean) ** 2
		z_mean_dev = (z_mean_dev / len(possible_ground_flat_sig)) ** 0.5
		n = len(possible_ground_flat_sig)
		i = 0
		while i < n:
			if possible_ground_flat_sig[i][5] - z_mean_mean > z_mean_dev:
				possible_ground_nonflat.append(possible_ground_flat[i])
				possible_ground_nonflat_sig.append(possible_ground_flat_sig[i])
				possible_ground_flat.pop(i)
				possible_ground_flat_sig.pop(i)
				n = n - 1
			else:
				i = i + 1



		print("number of possible_ground_flat_voxels: ", len(possible_ground_flat))
		print("number of possible_ground_nonflat_voxels: ", len(possible_ground_nonflat))


		# each ground-flat to plane
		for i in xrange(len(possible_ground_flat)):
			temp_cloud = [0]
			for j in xrange(len(possible_ground_flat[i])):
				temp_cloud.append([possible_ground_flat[i][j][3],possible_ground_flat[i][j][4],possible_ground_flat[i][j][5]])
			temp_cloud.pop(0)
			possible_ground_flat[i].insert(0,temp_cloud)
		possible_ground_flat_plane = [0]
		for i in xrange(len(possible_ground_flat)):
			temp_array = np.array(possible_ground_flat[i][0])
			possible_ground_flat[i].insert(0, Fit_Plane_SVD(temp_array))
			possible_ground_flat[i].pop(1)
			possible_ground_flat_plane.append(possible_ground_flat[i][0])
		possible_ground_flat_plane.pop(0)
		print("number of possible ground flat plane:", len(possible_ground_flat_plane))

		# universal ground : put all possible ground within 5000 mm range together 
		universal_possible_ground = []
		temp_cloud = []
		for i in xrange(len(possible_ground_flat)):
			if abs(possible_ground_flat_sig[i][4]) <= 5000 and abs(possible_ground_flat_sig[i][3]) <= 5000:
				universal_possible_ground = universal_possible_ground + possible_ground_flat[i][1:len(possible_ground_flat[i])-1]
		for i in xrange(len(universal_possible_ground)):
			temp_cloud = temp_cloud + [[universal_possible_ground[i][3],universal_possible_ground[i][4],universal_possible_ground[i][5]]]
		temp_array = np.array(temp_cloud)
		universal_possible_ground.insert(0,list(Fit_Plane_LTSQ(temp_array)))
		print("norm of universal ground:",universal_possible_ground[0])

	# def Object_Tracking(self, point_cloud, object_size):	# object size in volumn mm^3
		





		# # put near flat together
		# def If_Near_Voxels(v1,v2,xres,yres,zres):
		# 	v1x = v1[3]
		# 	v1y = v1[4]
		# 	v1z = v1[5]
		# 	v2x = v2[3]
		# 	v2y = v2[4]
		# 	v2z = v2[5]
		# 	if abs(v1x - v2x) <= 2*xres and abs(v1y - v2y) <= 2*yres and abs(v1z - v2z) <= 2*zres:
		# 		return True
		# 	else:
		# 		return False
		# def If_Near_Plane(p1,p2):
		# 	p1a = p1[0]
		# 	p1b = p1[1]
		# 	p1c = p1[2]
		# 	p2a = p2[0]
		# 	p2b = p2[1]
		# 	p2c = p2[2]
		# 	if p1a >= 0 and p2a >= 0:
		# 		a = True
		# 	else:
		# 		a = False
		# 	if p1b >= 0 and p2b >= 0:
		# 		b = True
		# 	else:
		# 		b = False
		# 	if p1c >= 0 and p2c >= 0:
		# 		c = True
		# 	else:
		# 		c = False
		# 	if a=b and b=c:
		# 		return True
		# 	else:
		# 		return False
		# temp_possible_ground_flat_sig = possible_ground_flat_sig
		# temp_possible_ground_flat_plane = possible_ground_flat_plane
		# temp_x_res = columb_res
		# temp_y_res = columb_res
		# temp_z_res = slice_res
		# for i in range(temp_possible_ground_flat_sig):		# add trace tag to flat_sig and flat_plane
		# 	temp_possible_ground_flat_sig[i] = [i,temp_possible_ground_flat_sig[i]]
		# 	temp_possible_ground_flat_plane[i] = [i,temp_possible_ground_flat_plane[i]]
		


		# g_num = []
		# while temp_possible_ground_flat_sig and temp_possible_ground_flat_plane:
		# 	g_num = g_num + [temp_possible_ground_flat_sig[0][0]]  
		# 	temp_possible_ground_flat_sig.pop(0)
		# 	temp_possible_ground_flat_plane.pop(0)
		
		# 	g_tag = 0
		
		# 	while g_tag == 0:
		# 		g_count = len(g_num) - 1

		# 		for n in range(len(g_num[g_count])):
		# 			n_n = g_num[g_count][n]


		# 			list_to_append = Near_Voxel_Recuisive()
					
					
		# 			m_tag = 0
					
		# 			for m in range(len(temp_possible_ground_flat_sig)):
		# 				if If_Near_Voxels(temp_possible_ground_flat_sig[n_n][1],temp_possible_ground_flat_sig[m][1],temp_x_res,temp_y_res,temp_z_res) \
		# 		   		and If_Near_Plane(temp_possible_ground_flat_plane[n_n][1],temp_possible_ground_flat_plane[m][1]):
		# 		   			g_num[g_count].append(temp_possible_ground_flat_sig[m][0])
		# 		   			m_list = m_list + [m]
		# 		   			m_tag = 1



		# 		   	for mm in len(m_list):
		# 	   			temp_possible_ground_flat_sig.pop(m_list[mm])
		# 	   			temp_possible_ground_flat_plane.pop(m_list[mm])


		# def Near_Voxel_Recuisive(close_voxel_list:initial_voxel, rest_voxel_list, tag):
		# 	if close voxel can be found:
		# 		Near_Voxel_Recuisive(initial_voxel, voxel_list, tag)
		# 	else:
		# 		return close_voxel_list



		# 	return near_voxel_list , loop_tag


		# # print len(possible_ground_flat_sig)
		# # print len(possible_ground_flat_plane)





		# x = [0]
		# y = [0]
		# z = [0]
		# fig = plt.figure()
		# test_pcl_plot = fig.gca(projection='3d')
		# test_pcl_plot.set_xlim3d(-10000, 10000)
		# test_pcl_plot.set_ylim3d(-10000, 10000)
		# test_pcl_plot.set_zlim3d(-10000, 10000)
		# for i in range(len(possible_ground_flat)):
		# 	for j in range(len(possible_ground_flat[i])):
		# 		x.append(possible_ground_flat[i][j][3])
		# 		y.append(possible_ground_flat[i][j][4])
		# 		z.append(possible_ground_flat[i][j][5])
		# test_pcl_plot.plot(x, y, z, '.')
		# x = [0]
		# y = [0]
		# z = [0]
		# # for i in range(len(possible_ground_nonflat)):
		# 	for j in range(len(possible_ground_nonflat[i])):
		# 		x.append(possible_ground_nonflat[i][j][3])
		# 		y.append(possible_ground_nonflat[i][j][4])
		# 		z.append(possible_ground_nonflat[i][j][5])
		# test_pcl_plot.plot(x, y, z, '*')
		# plt.show()



	
		






	def Build_Point_Voxel(self, FBPV_Points):
		init_point_cloud = FBPV_Points
		points_num = len(init_point_cloud)
		voxel_res = 500 # mm
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
			init_point_cloud[i] = init_point_cloud[i] # + xyz_id
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
