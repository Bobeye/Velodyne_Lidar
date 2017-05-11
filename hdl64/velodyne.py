import socket
import struct
import pickle

HOST = "192.168.3.77"
PORT = 2368

def read_lidar(ip, port):
	UDP_IP = ip
	UDP_PORT_LIDAR = port
	# lidar udp setup
	sock_lidar = socket.socket(socket.AF_INET, # Internet
					 socket.SOCK_DGRAM) # UDP

	sock_lidar.bind(('',PORT))

	print (sock_lidar)

	# sock_lidar.bind(('', UDP_PORT_LIDAR))

	# print (sock_lidar)
	packets = []
	for i in range(10000):
		data_lidar, addr_lidar = sock_lidar.recvfrom(1250)
		if len(data_lidar) == 1206:
			print (i, end = '\r')	
			packets += [data_lidar]

	with open('sample.pkl', 'wb') as f:
		pickle.dump(packets, f)


read_lidar(HOST, PORT)