import pickle
import struct
import calibration
import numpy as np
import csv



def save_csv(path, data):
	with open(path, 'w') as fp:
		wr = csv.writer(fp, delimiter=',')
		wr.writerows(data)

# load calibration parameters
cals = calibration.getCals()
print (len(cals))
for key in cals[0].keys():
	print (key)


# decode packets
with open('sample.pkl', 'rb') as p:
	packets = pickle.load(p)
print ('total number of packets: ', len(packets))
points = []
scans = []
azimuth_last = None
Nscan = 0
for ndata in range(3000):
	data_lidar = packets[ndata]
	timestamp, statusType, statusValue = struct.unpack_from("<Icc", data_lidar, offset=1200)
	# print (timestamp, chr(ord(statusType)), ord(statusValue))
	for offset in range(0, 1200, 100):
		firingData = struct.unpack_from('<ccH'+'hB'*32, data_lidar, offset)
		# print (firingData[0], firingData[1], firingData[2], firingData[36:40])
		azimuth = firingData[2]
		if azimuth_last is None:
			azimuth_last = azimuth
		if azimuth < azimuth_last:
			scans += [points]
			points = []
			Nscan += 1
			print (Nscan, ' scans extracted', end = '\r')

		# print (azimuth)
		if firingData[1] == b'\xee':
			# print ('upper')
			for lidar_id in range(32):
				distance = ((firingData[lidar_id*2+3])+cals[lidar_id]['distCorrection_']) / 100		# m
				RotAzi = azimuth / 100		# degree
				VertAngle = cals[lidar_id]['vertCorrection_'] 	# degree
				RotAngle = cals[lidar_id]['rotCorrection_']		# degree
				hOffsetCorr = cals[lidar_id]['horizOffsetCorrection_'] / 100 	# m
				vOffsetCorr = cals[lidar_id]['vertOffsetCorrection_'] / 100		# m

				if distance < 2:
					pass
				else:
					cosVertAngle = np.cos(np.deg2rad(VertAngle))
					sinVertAngle = np.sin(np.deg2rad(VertAngle))
					cosRotAngle = np.cos(np.deg2rad(RotAzi-RotAngle))
					sinRotAngle = np.sin(np.deg2rad(RotAzi-RotAngle))

					xyDistance = (distance * cosVertAngle) - (vOffsetCorr * sinVertAngle)
					x = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle
					y = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle
					z = distance * sinVertAngle + vOffsetCorr * cosVertAngle
					ite = firingData[lidar_id*2+4]
					pts = [x, y, z, ite]
					points += [pts]

		else:
			# print ('lower')
			for lidar_id in range(32):
				lidar_id += 32
				distance = ((firingData[(lidar_id-32)*2+3])+cals[lidar_id]['distCorrection_']) / 100		# m
				RotAzi = azimuth / 100		# degree
				VertAngle = cals[lidar_id]['vertCorrection_'] 	# degree
				RotAngle = cals[lidar_id]['rotCorrection_']		# degree
				hOffsetCorr = cals[lidar_id]['horizOffsetCorrection_'] / 100 	# m
				vOffsetCorr = cals[lidar_id]['vertOffsetCorrection_'] / 100		# m

				if distance < 2:
					pass
				else:
					cosVertAngle = np.cos(np.deg2rad(VertAngle))
					sinVertAngle = np.sin(np.deg2rad(VertAngle))
					cosRotAngle = np.cos(np.deg2rad(RotAzi))*np.cos(np.deg2rad(RotAngle)) + np.sin(np.deg2rad(RotAzi))*np.sin(np.deg2rad(RotAngle))
					sinRotAngle = np.sin(np.deg2rad(RotAzi))*np.cos(np.deg2rad(RotAngle)) - np.cos(np.deg2rad(RotAzi))*np.sin(np.deg2rad(RotAngle))

					xyDistance = distance * cosVertAngle - vOffsetCorr * sinVertAngle
					x = xyDistance * sinRotAngle - hOffsetCorr * cosRotAngle
					y = xyDistance * cosRotAngle + hOffsetCorr * sinRotAngle
					z = distance * sinVertAngle + vOffsetCorr * cosVertAngle
					ite = firingData[(lidar_id-32)*2+4]
					pts = [x, y, z, ite]
					points += [pts]

		azimuth_last = azimuth

# print (Nscan)

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

points = []
for p in range(len(scans[3])):
	if p % 1 == 0:
		x = scans[3][p][0]
		y = scans[3][p][1]
		z = scans[3][p][2]
		points.append([x,y,z])
points = np.array(points)
points = points.T

print (points.shape)

fig = plt.figure()
pts_plot = fig.gca(projection = '3d')
pts_plot.set_xlim3d(-20,20)
pts_plot.set_ylim3d(-20,20)
pts_plot.set_zlim3d(-2,10)	

pts_plot.plot(points[0], points[1], points[2], '.')
plt.show()


print ('saving csv')
for i in range(Nscan):
	points = []
	for p in range(len(scans[i])):
		x = scans[i][p][0]
		y = scans[i][p][1]
		z = scans[i][p][2]		
		points.append([x,y,z])

	csvName = 'sample_data/'+str("{0:0=6d}".format(i)) + '.csv'
	save_csv(csvName, points)
	print ('i', end = '\r')



