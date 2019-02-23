#!/usr/bin/env python

import rospy
import serial
import time

from std_msgs.msg import Float32MultiArray
from numpy import *
from easy_pyserial import *

ardu_byte_size = {
	"int": 2,
	"float": 4,
	"bool": 1,
	"char": 1,
	"byte": 1,
	"long": 4,
	"short": 2,
	"double": 4,
}

class SerialNode:
	def __init__(self, port, baud, timeout):
		#self.commands = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
		self.commands = [1.0, 3.0 ,5.0, 782.0, 13.0, 24.0, 240.0, 8.6]
		#self.commands = zeros(8).tolist()

		self.port = port
		self.baud = baud
		self.timeout = timeout

		self.ser = serial.Serial(port, baud, timeout = timeout)

	def begin(self, datatype, length):
		buffer_size = ardu_byte_size[datatype] * length
		ET = EasyTransfer(buffer_size, self.ser)

		return ET

	def main(self):
		self.ser.flushInput()
		self.ser.flushOutput()

		online = True
		counter = 0
		iterations = 1

		while not rospy.is_shutdown():
			if type(self.commands[0]) is not float:
				print ""
				raise Exception("Invalid float data. {} given instead".format(type(self.commands[0])))

			ETout.sendData(self.commands)
			#time.sleep(0.005)
			time.sleep(0.015)
			ETin.receiveData()

			print ETin.data

			if type(ETin.data[0]) is not float:
				print ""
				print "Failed to Establish Serial Communication!"
				print ""
				#raise Exception("Non float array received. {} received instead".format(type(ETin.data[0])))

			print ETin.data

			while not online and counter < iterations:
				ETout.sendData(self.commands)
				time.sleep(0.005)
				ETin.receiveData()
				time.sleep(0.1)

				print "Attempt {} out of {}".format(counter + 1, iterations), "\t", type(ETin.data[0])
				print "Serial Communication Offline."
				print "Restarting to try again."

				#self.close_and_open_port()
				#self.flush_serial_buffer()
				counter += 1

				if type(ETin.data[0]) is not float:
					print ""
					print "Failed to Establish Serial Communication!"
					print ""
					#raise Exception("Non float array received. {} received instead".format(type(ETin.data[0])))

			self.ser.flushInput()
			self.ser.flushOutput()		

			rospy.sleep(0)

if __name__ == '__main__':
	try:
		print 'Serial Node is Alive'
		rospy.init_node('serial_node', anonymous=True)

		port = '/dev/ttyACM0'
		baud = 115200
		timeout = 1

		SN = SerialNode(port, baud, timeout)

		time.sleep(0.5)

		ETout = SN.begin('float', 8)
		ETin = SN.begin('float', 24)

		SN.main()
	
	except rospy.ROSInterruptException:
		pass
