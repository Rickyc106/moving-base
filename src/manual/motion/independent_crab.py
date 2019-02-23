#!/usr/bin/env python

import os
import sys

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import rospy
from std_msgs.msg import Float32MultiArray
from math import sin, cos, atan2, sqrt, pi
from numpy import *
from datetime import *

from shared.joy_callback import *

class IndependentCrab:
	def __init__(self):
		self.JC = JoyCallback()
		self.current_time = datetime.now()
		self.prev = datetime.now()

		self.steer_position = zeros(4).tolist()
		self.drive_position = zeros(4).tolist()

	def mapf(self, x, in_min, in_max, out_min, out_max):
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def convert_controller_to_speed(self, input, throttle_cap):
		return throttle_cap * (input * 11.2)

	def controller_input(self):
		steer_input = 0.0

		if self.JC.left_bump and not self.JC.right_bump:
			steer_input = 1.0
		elif self.JC.right_bump and not self.JC.left_bump:
			steer_input = -1.0

		if self.JC.left_trig and not self.JC.right_trig:
			steer_input = self.mapf(self.JC.left_trig_axis, -1.0, 1.0, 1.0, 0.0)
		elif self.JC.right_trig and not self.JC.left_trig:
			steer_input = self.mapf(self.JC.right_trig_axis, -1.0, 1.0, -1.0, 0.0)

		drive_input = self.JC.y_axis

		steer_speed = self.convert_controller_to_speed(steer_input, 0.15)
		drive_speed =self.convert_controller_to_speed(drive_input, 0.05)

		self.current_time = datetime.now()
		delta_time = self.current_time - self.prev
		self.prev = datetime.now()

		if not any(self.JC.module_buttons):
			self.JC.motor_stop()

		for module in range(4):
			if (self.JC.module_buttons[module]):

				self.steer_position[module] += steer_speed * delta_time.total_seconds()
				self.drive_position[module] += drive_speed * delta_time.total_seconds()

				self.JC.stop_msg.data[module * 2] = 0.0
				self.JC.stop_msg.data[module * 2 + 1] = 0.0

				self.JC.motor_msg.data[module * 2] = self.steer_position[module]
				self.JC.motor_msg.data[module * 2 + 1] = self.drive_position[module]

	def main(self):
		print self.JC.module_found
		print "Starting Joy Callback Node..."
		self.JC.subscribe()

		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			self.controller_input()
			self.JC.publish()
			rate.sleep()

if __name__ == '__main__':
	try:
		print "Independent Crab Node is Alive!"
		rospy.init_node('independent_crab_node', anonymous=True)

		IC = IndependentCrab()
		IC.main()
	except rospy.ROSInterruptException:
		pass