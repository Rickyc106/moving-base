#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from math import sin, cos, atan2, sqrt, pi
from numpy import *

class ComplexCrab:
	def __init__(self):
		self.resultant_angle = [0,0,0,0]
		self.resultant_mag = [0,0,0,0]

		self.rotate_angle = [0,0,0,0]
		self.rotate_mag = [0,0,0,0]

		self.translate_angle = 0.0
		self.translate_mag = 0.0

		self.resultant_angle_msg, self.resultant_mag_msg, self.motor_msg = Float32MultiArray(), Float32MultiArray(), Float32MultiArray()
		self.resultant_angle_msg.data, self.resultant_mag_msg.data, self.motor_msg.data = zeros(4), zeros(8), zeros(8)

	def vector_sum(self, steer_angle, steer_mag, rotate_angle, rotate_mag, index):
		res_x = (steer_mag * cos(steer_angle)) + (rotate_mag * cos(rotate_angle))
		res_y = (steer_mag * sin(steer_angle)) + (rotate_mag * sin(rotate_angle))

		self.resultant_angle[index] = atan2(res_y, res_x)
		self.resultant_mag[index] = sqrt(res_x**2 + res_y**2)

	def main(self):
		rotate_angle, rotate_mag = zeros(4), zeros(4)

		if (msg.buttons[9]): # Start Button changes
			self.rotate_and_translate = True

		if ((msg.buttons[4] and not msg.buttons[5]) or (msg.buttons[6] and not msg.buttons[7])):
			self.CCW = True
			self.no_rotate = False

		if ((msg.buttons[5] and not msg.buttons[4]) or (msg.buttons[7] and not msg.buttons[6])):
			self.CCW = False
			self.no_rotate = False

		if (not msg.buttons[4] and not msg.buttons[5] and not msg.buttons[6] and not msg.buttons[7]):
			self.no_rotate = True

		steer_angle = atan2(msg.axes[1], -msg.axes[0])
		
		x = cos(steer_angle) * -msg.axes[0]
		y = sin(steer_angle) * msg.axes[1]

		steer_mag = sqrt(x**2 + y**2)

		for i in range(self.modules):
			if (self.CCW and not self.no_rotate):
				self.rotate_angle[i] += pi / 22.5;
				rotate_mag[i] = 1.0

				steer_input = self.mapf(msg.axes[2], -1.0, 1.0, 1.0, 0.0)
			elif (not self.CCW and not self.no_rotate):
				self.rotate_angle[i] -= pi / 22.5;
				rotate_mag[i] = 1.0

				steer_input = self.mapf(msg.axes[2], -1.0, 1.0, -1.0, 0.0)
			
			if (i == 0 or i == 1):
				rotate_mag[i] = -1.0

			if (self.no_rotate):
				rotate_mag[i] = 0.0
				steer_input = 0.0

			if (steer_mag != 0.0 or rotate_mag[i] != 0):
				self.vector_sum(steer_angle, steer_mag, self.rotate_angle[i], rotate_mag[i], i)

		self.resultant_angle_msg.data = self.resultant_angle
		self.resultant_mag_msg.data = self.resultant_mag

		drive_input = msg.axes[1] * 0.4

		if (not msg.buttons[0]):
			self.motor_stop()

		#if (not msg.buttons[0] and not msg.buttons[1] and not msg.buttons[2] and not msg.buttons[3]):
		#	self.motor_stop()

		#for module in range(self.modules):
		#	if (msg.buttons[module]):
		#		self.motor_msg.data[module * 2] = steer_input
		#		self.motor_msg.data[module * 2 + 1] = drive_input

		for module in range(self.modules):
			self.motor_msg.data[module * 2] = self.resultant_angle[module]
			self.motor_msg.data[module * 2 + 1] = self.resultant_mag[module]

	def init(self):
		self.rotate_angle[0] -= pi / 4
		self.rotate_angle[1] += pi / 4
		self.rotate_angle[2] -= pi / 4
		self.rotate_angle[3] += pi / 4
