#!/usr/bin/python

import os
import csv
import sys
import dpkt
import socket
import glob
from datetime import datetime, timedelta
import struct
import time
import traceback
import numpy as np
from multiprocessing import Process, Queue, Pool
import gc

import logging
import logging.config

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
# from scipy import ndimage
# from sklearn.cluster import DBSCAN
# from sklearn import metrics
# from sklearn.datasets.samples_generator import make_blobs
# from sklearn.preprocessing import StandardScaler

HOST = "192.168.1.201"
PORT = 2368

LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1
DISTANCE_RESOLUTION = 0.002
ROTATION_RESOLUTION = 0.01
ROTATION_MAX_UNITS = 36000

DATA_QUEUE = Queue(-1)

PTS_QUEUE = Queue(-1)

formatter = '[%(asctime)s][%(filename)s:%(lineno)s][%(levelname)s][%(message)s]'

LOGGING_CONFIG = {
	'version': 1,
	'disable_existing_loggers': False,  # this fixes the problem

	'formatters': {
		'standard': {
			'format': formatter,
		},
	},
	'handlers': {
		'default': {
			'level': 'DEBUG',
			'class': 'logging.StreamHandler',
			'formatter': 'standard'
		},
		"debug_file_handler": {
			"class": "logging.handlers.TimedRotatingFileHandler",
			"level": "DEBUG",
			"formatter": "standard",
			"filename": "./logs/lidar.log",
			"when": "D",
			"interval": 1,
			"backupCount": 30,
			"encoding": "utf8"
		},
	},
	'loggers': {
		'': {
			'handlers': ["default", 'debug_file_handler'],
			'level': 'DEBUG',
			'propagate': False
		},
	}
}

# logging.config.dictConfig(LOGGING_CONFIG)
# logger = logging.getLogger("")

def save_csv(path, data):
	with open(path, 'w') as fp:
		wr = csv.writer(fp, delimiter=',')
		wr.writerows(data)

def calc(dis, azimuth, laser_id, timestamp):
	R = dis * DISTANCE_RESOLUTION
	omega = LASER_ANGLES[laser_id] * np.pi / 180.0
	alpha = azimuth / 100.0 * np.pi / 180.0
	X = R * np.cos(omega) * np.sin(alpha)
	Y = R * np.cos(omega) * np.cos(alpha)
	Z = R * np.sin(omega)
	return [X, Y, Z, R, omega, alpha, timestamp]

def unpack(dirs):
	files = glob.glob(dirs + '/*.bin')
	points = []
	prev_azimuth = None



	for x in files:
		d = open(x, 'rb').read()
		n = len(d)
		for offset in xrange(0, n, 1223):
			ts = d[offset : offset + 17]
			data = d[offset + 17 : offset + 1223]
			print (ts, len(data))
			timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
			assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
			timestamp = float(ts)
			seq_index = 0
			for offset in xrange(0, 1200, 100):
				flag, azimuth = struct.unpack_from("<HH", data, offset)
				assert flag == 0xEEFF, hex(flag)
				for step in xrange(2):
					seq_index += 1
					azimuth += step
					azimuth %= ROTATION_MAX_UNITS
					if prev_azimuth is not None and azimuth < prev_azimuth:
						path = dirs
						try:
							if os.path.exists(path) is False:
								os.makedirs(path)
						except Exception, e:
							print e
						if not points:
							timestamp_str = '%.6f' % time.time()
						else:
							timestamp_str = '%.6f' % points[0][6]
						save_csv("{}/{}.csv".format(path, timestamp_str), points)
						print ('.')
						# logger.info("{}/i{}_{}.csv".format(path, csv_index, timestamp_str))
						points = []
					prev_azimuth = azimuth
					# H-distance (2mm step), B-reflectivity (0
					arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
					for i in xrange(NUM_LASERS):
						time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
						if arr[i * 2] != 0:
							points.append(calc(arr[i * 2], azimuth, i, timestamp + time_offset))

def unpack_pcap(dirs):
	pcap_files = glob.glob(dirs + '/*.pcap')
	points = []
	prev_azimuth = None
		
	for pcap in pcap_files:
		pname = pcap.split('.')[0] + '_'
		pcap_file = open(pcap)				# import the .pcap file
		pcap = dpkt.pcap.Reader(pcap_file)		# read the .pcap file using dpkt
		# save the .pcap to buffer
		for ts, buf in pcap:
			udpc = buf 		# current udp packet
			eth = dpkt.ethernet.Ethernet(udpc)
			ip = eth.data
			udp = ip.data
			data = udp.data		
			timeblock = data[1200:1204]
			timestamp = (((ord(timeblock[3])<<8) | (ord(timeblock[2])))<<16) | (((ord(timeblock[1])<<8) | (ord(timeblock[0]))))
			timestamp = float(timestamp/1000000.0)
			seq_index = 0
			for offset in xrange(0, 1200, 100):
				flag, azimuth = struct.unpack_from("<HH", data, offset)
				assert flag == 0xEEFF, hex(flag)
				for step in xrange(2):
					seq_index += 1
					azimuth += step
					azimuth %= ROTATION_MAX_UNITS
					if prev_azimuth is not None and azimuth < prev_azimuth:
						path = dirs
						try:
							if os.path.exists(path) is False:
								os.makedirs(path)
						except Exception, e:
							print e
						if not points:
							timestamp_str = '%.6f' % time.time()
						else:
							timestamp_str = '%.6f' % points[0][6]
						save_csv("{}/{}.csv".format(path, timestamp_str), points)
						print (timestamp_str)
						# logger.info("{}/i{}_{}.csv".format(path, csv_index, timestamp_str))
						points = []
					prev_azimuth = azimuth
					# H-distance (2mm step), B-reflectivity (0
					arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
					for i in xrange(NUM_LASERS):
						time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
						if arr[i * 2] != 0:
							points.append(calc(arr[i * 2], azimuth, i, timestamp + time_offset))







def capture_bin(bin_folder, pts_queue):



	try:
		files = glob.glob(bin_folder + '/*.bin')
		points = []
		prev_azimuth = None
		for x in files:
			d = open(x, 'rb').read()
			n = len(d)

			print ("Start broadcast points")

			for offset in xrange(0, n, 1223):
				ts = d[offset : offset + 17]
				data = d[offset + 17 : offset + 1223]
				# print ts, len(data)
				timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
				assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
				timestamp = float(ts)
				seq_index = 0
				for offset in xrange(0, 1200, 100):
					flag, azimuth = struct.unpack_from("<HH", data, offset)
					assert flag == 0xEEFF, hex(flag)
					for step in xrange(2):
						seq_index += 1
						azimuth += step
						azimuth %= ROTATION_MAX_UNITS
						if prev_azimuth is not None and azimuth < prev_azimuth:
							if not points:
								timestamp_str = '%.6f' % time.time()
							else:
								timestamp_str = '%.6f' % points[0][6]
							pts = np.array(points)




							pts_queue.put({'points': pts, 'time': timestamp_str})
							# logger.info("{}/i{}_{}.csv".format(path, csv_index, timestamp_str))
							points = []
						prev_azimuth = azimuth
						# H-distance (2mm step), B-reflectivity (0
						arr = struct.unpack_from('<' + "HB" * 16, data, offset + 4 + step * 48)
						
						# pts_16 = np.empty([16,7]).astype(np.float64)
						# print "sss" 
						# initial_ground = 0
						# ground = True
						for i in xrange(NUM_LASERS):
							# if i % 2 == 0:
							# 	n = i / 2
							# else:
							# 	n = 7+(i+1)/2							
							time_offset = (55.296 * seq_index + 2.304 * i) / 1000000.0
							if arr[i * 2] != 0:

								# temp_pts = calc(arr[i * 2], azimuth, i, timestamp + time_offset)
								# if abs(temp_pts[0]) >= 2 and abs(temp_pts[1]) >= 2:
								# 	if not ground:
								# 		points.append(temp_pts)
								# 	else:
								# 		if initial_ground == 0 and temp_pts[2] <= -1.4:
								# 			initial_ground = temp_pts[2]
								# 		elif abs(temp_pts[2] - initial_ground) <= 1:
								# 			initial_ground = temp_pts[2]
								# 		else:
								# 			points.append(temp_pts)
								# 			ground = False
								points.append(calc(arr[i * 2], azimuth, i, timestamp + time_offset))
	except KeyboardInterrupt:
		gc.collect()
		print "end"
			
def tracking(output, pts_queue):
	t1 = time.time()
	try:
		while True:
			if pts_queue.empty():
				pass
			else:
				msg = pts_queue.get()
				pts = msg['points']
				t = msg['time']

				# pts = pts.T[:, pts.T[3] < 30].T

				print pts.shape


				if pts.shape[0] > 0:			
					# pts = pts[~np.all(pts[3] > 30, axis = 0)]

					# pts = pts[:,pts[3]<=30]
					pts = pts.T[:, pts.T[3] < 30].T
					# pts = pts.T[:, pts.T[2] > -1.5].T
					# # pts = pts.T[:, pts.T[3] > 2].T
					# # pts = pts.T[:, pts.T[2] < 1].T


					# pts_depth = np.empty([16*3600,7]).astype(np.int64)
					# for pt in pts:
					# 	if pt[]









					# print pts.shape


					# # pts_xyz = pts_xyz[~np.all(pts_xyz == 0, axis=1)]

					pts_xyz = pts.T[0:3].T
					# pts_xy = pts.T[0:2].T



					# # pts_depth = np.empty([16*360,10]).astype(np.int64)


					# print pts_xyz.shape

					# # rough cluster -- kd-tree
					# DIM = 3
					# from scipy.spatial import KDTree
					# tree = KDTree(pts_xyz, leafsize=pts_xyz.shape[0]+1)

					

					# DBSCAN
					db = DBSCAN(eps=0.5, min_samples=20, algorithm='auto', n_jobs=8).fit(pts_xyz)
					core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
					core_samples_mask[db.core_sample_indices_] = True
					labels = db.labels_

					# Number of clusters in labels, ignoring noise if present.
					n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
					print('Estimated number of clusters: %d' % n_clusters_)


					fig = plt.figure()
					pts_plot = fig.gca(projection = '3d')
					pts_plot.set_xlim3d(-20,20)
					pts_plot.set_ylim3d(-20,20)
					pts_plot.set_zlim3d(-10,10)	
					

					unique_labels = set(labels)


					colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
					for k, col in zip(unique_labels, colors):
						# if k == -1:
						# 	# Black used for noise.
						# 	col = 'k'

						class_member_mask = (labels == k)



						xyz = pts_xyz[class_member_mask]
						
						# if xyz.shape[0] > 1000:
						# 	pass
						# else:
						plt.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], 'o', markerfacecolor=col, markeredgecolor='k', markersize=2)


						
					plt.savefig("test.png")
					

				
				t2 = time.time()
				print t2 - t1
			
				t1 = t2
	except KeyboardInterrupt:
		gc.collect()
		print "end"





def save_package(dirs, data_queue):
	# fp = None
	file_fmt = os.path.join(dirs, '%Y-%m-%d_%H%M')
	binpath = str(datetime.now().strftime(file_fmt)) + '.bin'
	try:
		if os.path.exists(dirs) is False:
			print dirs
			os.makedirs(dirs)
		cnt = 0
		fp = None
		while True:
			if data_queue.empty():
				pass
			else:
				msg = data_queue.get()
				data = msg['data']
				ts = msg['time']
				print ts, len(data), 'queue size: ', data_queue.qsize(), cnt
				if fp == None or cnt == 1000000:
					if fp != None:
						fp.close()
					path = binpath
					# logger.info('save to' + path)
					print 'save to ', path
					fp = open(path, 'ab')
					cnt = 0
				cnt += 1
				fp.write('%.6f' % ts)
				fp.write(data)
	except KeyboardInterrupt, e:
		print e
	finally:
		fp.close()

def capture(port, data_queue):
	soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	soc.bind(('', port))
	try:
		while True:
			try:
				data = soc.recv(2000)
				if len(data) > 0:
					assert len(data) == 1206, len(data)
					data_queue.put({'data': data, 'time': time.time()})
			except Exception, e:
				print dir(e), e.message, e.__class__.__name__
				traceback.print_exc(e)
	except KeyboardInterrupt, e:
		print e

def pcap_capture(pcap_folder, data_queue):
	pcap_files = glob.glob(pcap_folder + '/*.pcap')
	print pcap_files
	for pcap in pcap_files:
		pcap_file = open(pcap)				# import the .pcap file
		pcap = dpkt.pcap.Reader(pcap_file)		# read the .pcap file using dpkt
		# save the .pcap to buffer
		for ts, buf in pcap:

			udpc = buf 		# current udp packet
			eth = dpkt.ethernet.Ethernet(udpc)
			ip = eth.data
			udp = ip.data
			data = udp.data
			assert len(data) == 1206, len(data)
			data_queue.put({'data': data, 'time': time.time()})




if __name__ == "__main__":
	if sys.argv[1] == 'pcap':
		top_dir = datetime.now().strftime('%Y-%m-%d_%H%M%S')
		pcap_folder = sys.argv[2]
		processA = Process(target = pcap_capture, args = (pcap_folder , DATA_QUEUE))
		processA.start()
		processB = Process(target = save_package, args = (top_dir, DATA_QUEUE))
		processB.start()
		
	elif sys.argv[1] == 'tracking':
		bin_folder = sys.argv[2]
		processA = Process(target = capture_bin, args = (bin_folder , PTS_QUEUE))
		processA.start()
		processB = Process(target = tracking, args = (3, PTS_QUEUE))
		processB.start()
	
	elif sys.argv[1] == 'unpack':
		bin_folder = sys.argv[2]
		unpack(bin_folder)

	elif sys.argv[1] == 'unpack_pcap':
		pcaps_folder = sys.argv[2]
		unpack_pcap(pcaps_folder)
