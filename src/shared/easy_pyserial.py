#!/usr/bin/env python

import struct
import sys
import serial
from numpy import *

class EasyTransfer:
	def __init__(self, length, _Serial):
		self.size = length
		self.ser = _Serial
		self.data = zeros(length // 4).tolist()

	def sxor(self, s1, s2):    
	    # convert strings to a list of character pair tuples
	    # go through each tuple, converting them to ASCII code (ord)
	    # perform exclusive or on the ASCII code
	    # then convert the result back to ASCII (chr)
	    # merge the resulting array of characters as a string
	    return ''.join(chr(ord(a) ^ ord(b)) for a,b in zip(s1,s2))

	def sendData(self, msg):
		CS = self.size

		# Send 3 Bytes - 1st Start Bit
		self.ser.write(struct.pack('B', 0x06))
		# Check next bit after 1st start bit
		self.ser.write(struct.pack('B', 0x85))
		# Send length
		self.ser.write(struct.pack('B', self.size))

		CS = struct.pack('B', CS)
		buf = struct.pack('%sf' % len(msg), *msg)

		for i in range(self.size):
			# Bitwise XOR checksum
			CS = self.sxor(CS, buf[i])
			# Send message data
			self.ser.write(buf[i])

		# Send final checksum
		self.ser.write(CS)

	def receiveData(self):
		rx_len = 0
		rx_array_index = 0
		buf = ""

		# Wait for first 3 bytes of transmission first -- Why????
		#if self.ser.in_waiting >= 3:
		#	#print "Found something"
		#	# Wait for start bit -- vvv re-indent if needed

		while self.ser.read() != struct.pack('B', 0x06):
			# If less than 3 bytes, loop back
			if self.ser.in_waiting < 3:
				#return False
				pass
		# Wait for next bit
		if self.ser.read() == struct.pack('B', 0x85):
			# Set length of data
			rx_len = struct.unpack('B', self.ser.read())[0]
			# Check if length matches with size
			if(rx_len != self.size):
				rx_len = 0
				return False
			else:
				#print "Warning: Transmission Size is not Equal!"
				pass
		else:
			#print "Warning: Byte(0x85) not Found!"
			pass

		#else:
		#	#print "Warning: Less than three bytes received! {} bytes received.".format(self.ser.in_waiting)
		#	pass

		# Wait for length to be set
		if rx_len != 0:
			# Loop through bytes
			counter = 0
			while self.ser.in_waiting and rx_array_index <= rx_len:
				# Store in buffer for now
				buf += self.ser.read()
				rx_array_index += 1
				counter += 1
			# Check if all bytes were sent
			if rx_len == (rx_array_index - 1):
				calc_CS = struct.pack('B', rx_len)
				# Calculate checksum
				for i in range(rx_len):
					calc_CS = self.sxor(calc_CS, buf[i])
				# Check if checksum matches with transmitted checksum
				if calc_CS == buf[rx_array_index - 1]:
					# Success! Set data to buffer
					#print "Success!"
					self.data = list(struct.unpack('%sf' % (len(buf) // 4), buf[:-1]))
					# Reset all helper variables
					rx_len = 0
					rx_array_index = 0

					return True
				else:
					rx_len = 0
					rx_array_index = 0

					print "Warning: Failed checksum!"

					return False
			else:
				#print "Warning: Transmission size is not Equal!"
				pass
		else:
			#print "Warning: RX Serial Length not set!"
			pass

		return False

