def time_append_hex_1(a, b):
	sizeof_b = 0
	# get size of b in bits
	while((b >> sizeof_b) > 0):
		sizeof_b += 1
	# align answer to nearest 4 bits (hex digit)
	sizeof_b += sizeof_b % 4

	print ("b size",sizeof_b)

	return (a << sizeof_b) | b

def time_append_hex_2(a, b):
	sizeof_b = 0
	# get size of b in bits
	while((b >> sizeof_b) > 0):
		sizeof_b += 1
	# align answer to nearest 4 bits (hex digit)
	sizeof_b += sizeof_b % 6

	print ("b size",sizeof_b)

	return (a << sizeof_b) | b






time_block = ['o', '?', '\r', '\x93']


time_stamp_1 = time_append_hex_1(ord(time_block[3]),ord(time_block[2]))
print hex(time_stamp_1)
time_stamp_2 = time_append_hex_1(ord(time_block[1]),ord(time_block[0]))
print hex(time_stamp_2)
time_stamp = time_append_hex_2(hex(time_stamp_1),hex(time_stamp_2))
print hex(time_stamp)
print time_stamp




