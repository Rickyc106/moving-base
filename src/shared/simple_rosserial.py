#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float32MultiArray

def encoder_cb(msg):
	print msg.data

def main():
	command_pub = rospy.Publisher('commands', Float32MultiArray, queue_size = 10)

	rospy.init_node('simple_rosserial_node', anonymous=True)
	rospy.Subscriber('encoder_ticks', Float32MultiArray, encoder_cb)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		command_array = Float32MultiArray()

		for i in range(8):
			command_array.data.append(i)

		command_pub.publish(command_array)
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass