import socket
import struct


UDP_IP = "192.168.1.77"
UDP_PORT = 2368

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind(('', UDP_PORT))

buf_l = [0]

for i in range(1000):
	data, addr = sock.recvfrom(1248) # buffer size is 1024 bytes
	buf_l.append(data)

	print repr(data)

buf_l.pop(0)
print len(buf_l)