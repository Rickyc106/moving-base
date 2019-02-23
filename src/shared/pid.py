#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from numpy import *

class PID:
	def __init__(self, p_gain, i_gain, d_gain, integral_limit):
		self.p_gain = p_gain
		self.i_gain = i_gain
		self.d_gain = d_gain

		self.integral_limit = integral_limit

		self.motor_desired, self.motor_actual = zeros(8).tolist(), zeros(8).tolist()
		self.error, self.pid_sum = zeros(8).tolist(), zeros(8).tolist()

		self.motor_msg = Float32MultiArray()
		self.motor_msg.data = zeros(8).tolist()

	def desired_motor(self, msg):
		self.motor_desired = msg.data

	def actual_motor(self, msg):
		self.motor_actual = msg.data

	def constrain(self, value, lo, hi):
		if value < lo:
			value = lo
		elif value > hi:
			value = hi
		else:
			pass

		return value

	def PID_controller(self, index, desired, actual):
		for motor in range(8):
			if index == motor:
				self.error[index] = desired - actual
				
				self.pid_sum[index] += self.error[index]
				self.pid_sum[index] = self.constrain(self.pid_sum[index], -self.integral_limit, self.integral_limit)

				output = self.p_gain * self.error[index] + self.i_gain * self.pid_sum[index] + self.d_gain * (self.error[index] - self.error[index])
		
		return output

	def main(self):
		rate = rospy.Rate(30)

		while not rospy.is_shutdown():
			for motor in range(8):
				output = self.PID_controller(motor, self.motor_desired[motor], self.motor_actual[motor])
				output = self.constrain(output, -1.0, 1.0)				
				self.motor_msg.data[motor] = output

			motor_pub.publish(self.motor_msg)
			rate.sleep()

if __name__ == '__main__':
	try:
		print "PID Controller is Alive!"
		rospy.init_node('pid_controller', anonymous=True)
		motor_pub = rospy.Publisher('commands', Float32MultiArray, queue_size=10)
		
		manual_mode = rospy.get_param('/pid_controller/manual_mode')
		
		p_gain = rospy.get_param('/pid_controller/p_gain')
		i_gain = rospy.get_param('/pid_controller/i_gain')
		d_gain = rospy.get_param('/pid_controller/d_gain')

		integral_limit = rospy.get_param('/pid_controller/integral_limit')
		pid = PID(p_gain, i_gain, d_gain, integral_limit)

		if manual_mode:
			rospy.Subscriber('desired_position', Float32MultiArray, pid.desired_motor)
		else:
			rospy.Subscriber('fake_topic', Float32MultiArray, pid.desired_motor)

		rospy.Subscriber('encoder_position', Float32MultiArray, pid.actual_motor)
		
		pid.main()
	except rospy.ROSInterruptException:
		pass