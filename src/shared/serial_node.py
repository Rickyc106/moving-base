#!/usr/bin/env python

import rospy
import serial
import time
import struct
import sys
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
		self.commands = zeros(8).tolist()

		self.port = port
		self.baud = baud
		self.timeout = timeout

		self.ser = serial.Serial(port, baud, timeout = timeout)
		self.first_loop = True

		self.encoder_ticks = Float32MultiArray()
		self.encoder_period = Float32MultiArray()
		self.encoder_position = Float32MultiArray()

		self.encoder_ticks.data = zeros(8).tolist()
		self.encoder_period.data = zeros(8).tolist()
		self.encoder_position.data = zeros(8).tolist()

		self.motor_commands = zeros(8).tolist()

	def commandsCallback(self, msg):
		self.commands = msg.data

	def begin(self, datatype, length):
		buffer_size = ardu_byte_size[datatype] * length
		ET = EasyTransfer(buffer_size, self.ser)

		return ET

	def validate_checksum(self):
		test_msg = [1.0, 2.0, 3.0, 4.0, 5.0]
		ETout.sendData(test_msg)
		ETin.receiveData()

		if test_msg.all() == ETin.data[:,5]:
			print "Serial Communication Online"
			return True

		return False

	def close_and_open_port(self):
		self.ser.flushInput()
		self.ser.flushOutput()

		self.ser.close()
		time.sleep(6)

		self.ser.open()

		self.ser = serial.Serial(self.port, self.baud, timeout = self.timeout)
		time.sleep(0.5)

	def flush_serial_buffer(self):
		while self.ser.in_waiting > 0:
			char = self.ser.read()
			time.sleep(0.01)

	def main(self):
		self.ser.flushInput()
		self.ser.flushOutput()

		online = False
		counter = 0
		iterations = 1

		#while not self.validate_checksum():
		#	print "Establishing Serial Communication."

		while not rospy.is_shutdown():
			if type(self.commands[0]) is not float:
				print ""
				raise Exception("Invalid float data. {} given instead".format(type(self.commands[0])))

			ETout.sendData(self.commands)
			time.sleep(0.005)
			ETin.receiveData()

			if type(ETin.data[0]) is not float:
				print ""
				print "Failed to Establish Serial Communication!"
				print ""
				raise Exception("Non float array received. {} received instead".format(type(ETin.data[0])))

			#ticks = ETin.data[:8]
			#period = ETin.data[8:16]
			#pos = ETin.data[16:24]
			#debug = ETin.data[24:]

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
					raise Exception("Non float array received. {} received instead".format(type(ETin.data[0])))

			self.ser.flushInput()
			self.ser.flushOutput()		

			rospy.sleep(0)

if __name__ == '__main__':
	try:
		print "Serial Node is Alive!"
		rospy.init_node('serial_node', anonymous=True)

		port = rospy.get_param('/serial_node/port')
		baud = rospy.get_param('/serial_node/baud')
		timeout = rospy.get_param('/serial_node/timeout')

		#SN = SerialNode(port, baud, timeout)
		SN = SerialNode('/dev/ttyACM0', 115200, 0.5)

		time.sleep(0.5)

		ETout = SN.begin("float", 8)
		ETin = SN.begin("float", 8)

		rospy.Subscriber('commands', Float32MultiArray, SN.commandsCallback)
		SN.main()

		#while not rospy.is_shutdown():
		#	ETout.sendData(SN.commands)
		#	ETin.receiveData()
#
#			print ETin.data
#			print ""

	except rospy.ROSInterruptException:
		pass