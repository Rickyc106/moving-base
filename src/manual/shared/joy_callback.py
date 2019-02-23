#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from numpy import *

class JoyCallback:
	def __init__(self):
		self.module_found = "Joy Callback Module Found!"

		self.x_axis = 0.0
		self.y_axis = 0.0

		self.left_trig_axis = 0.0
		self.right_trig_axis = 0.0

		self.left_bump, self.right_bump = 0.0, 0.0
		self.left_trig, self.right_trig = 0.0, 0.0

		self.x = 0.0
		self.circle = 0.0
		self.tri = 0.0
		self.square = 0.0

		self.module_buttons = zeros(4).tolist()
		self.start = 0.0

		self.motor_pub = rospy.Publisher('desired_position', Float32MultiArray, queue_size=10)
		self.motor_msg = Float32MultiArray()
		self.motor_msg.data = zeros(8).tolist()

		self.motor_stop_pub = rospy.Publisher('stop_commands', Float32MultiArray, queue_size=10)
		self.stop_msg = Float32MultiArray()
		self.stop_msg.data = zeros(8).tolist()

		self.reset_position = [5.149620947, 2.061255939, 2.650718801, 1.377136505]

	def motor_stop(self):
		for motor in range(8):
			self.stop_msg.data[motor] = 1.0

	def motor_reset(self):
		for motor in range(8):
			if not motor % 2:
				self.motor_msg.data[motor] = self.reset_position[motor]
			elif motor % 2:
				self.stop_msg.data[motor] = 1.0

	def joy_cb(self, msg):
		self.x_axis = msg.axes[0]
		self.y_axis = msg.axes[1]

		self.left_trig_axis = msg.axes[2]
		self.right_trig_axis = msg.axes[5]

		self.left_bump, self.right_bump = msg.buttons[4], msg.buttons[5]
		self.left_trig, self.right_trig = msg.buttons[6], msg.buttons[7]

		self.x = msg.buttons[0]
		self.circle = msg.buttons[1]
		self.tri = msg.buttons[2]
		self.square = msg.buttons[3]

		self.module_buttons = [self.x, self.circle, self.tri, self.square]

	def subscribe(self):
		rospy.Subscriber('joy', Joy, self.joy_cb)
		print "I am subscribing"

	def publish(self):
		self.motor_pub.publish(self.motor_msg)
		self.motor_stop_pub.publish(self.stop_msg)

	def main(self):
		self.subscribe()
		self.publish()

		while not rospy.is_shutdown():
			#resultant_angle_pub.publish(self.resultant_angle_msg)
			#resultant_mag_pub.publish(self.resultant_mag_msg)
			pass

if __name__ == '__main__':
	try:
		print "Joy Callback is Alive!"
		rospy.init_node('joy_callback_node', anonymous=True)

		#resultant_angle_pub = rospy.Publisher('resultant_angle', Float32MultiArray, queue_size=10)
		#resultant_mag_pub = rospy.Publisher('resultant_mag', Float32MultiArray, queue_size=10)
	except rospy.ROSInterruptException:
		pass
