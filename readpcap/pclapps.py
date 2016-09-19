import numpy as np
import pcl

class PCL_APPS(object):
	def __init__(self, pclapps_point_cloud):
		self.__point_cloud = pclapps_point_cloud

	def Build_Point_Cloud(self):
		init_point_cloud = self.__point_cloud
		points_num = len(init_point_cloud)
		# build pck format of point cloud: points
		points_list = [[init_point_cloud[0][3],init_point_cloud[0][4],init_point_cloud[0][5]]]
		for i in range(points_num-1):
			points_list.append([init_point_cloud[i+1][3],init_point_cloud[i+1][4],init_point_cloud[i+1][5]])
		points_array = np.array(points_list,dtype=np.float32)
		points = pcl.PointCloud()
		points.from_array(points_array)
		points = pcl.PointCloud(points_array)

		return points

	def K_D_Tree(self):
		cloud = self.Build_Point_Cloud()

		kd = cloud.make_kdtree_flann()

		print kd

	def Filter(self):
		cloud = self.Build_Point_Cloud()

		mls = cloud.make_moving_least_squares()
		mls.set_search_radius(0.5)
		mls.set_polynomial_fit(True)

		f = mls.process()

		print f

		# new instance is returned
		f.assertNotEqual(pcl.cloud, f)
		# mls filter retains the same number of points
		f.assertEqual(pcl.cloud.size, f.size)

		print f

        # f = mls.process()
  #       # new instance is returned
  #       pcl.assertNotEqual(pcl.cloud, f)
  #       # mls filter retains the same number of points
  #       pcl.assertEqual(pcl.cloud.size, f.size)





