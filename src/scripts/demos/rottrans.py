#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import time
import sys

prev_theta = float( sys.argv[1] )

def rotAndTranslate(angle, stspeed, drspeed):
	steer_speed = stspeed
	drive_speed = drspeed

	pub = rospy.Publisher('MOTOR_ACT', Float32MultiArray, queue_size=12)
	rospy.init_node("talker", anonymous=True)
	rate = rospy.Rate(100) #100Hz
	msg = Float32MultiArray()

	msg.data = [prev_theta, prev_theta+0.83, prev_theta, prev_theta+0.33,
	 steer_speed, steer_speed, steer_speed, steer_speed,
	 drive_speed, drive_speed, drive_speed, drive_speed];
	
	rospy.loginfo(msg)
	pub.publish(msg)


if __name__ == '__main__':
	try:
		print(prev_theta)
		rotAndTranslate(prev_theta, 1, 2.4)
	except rospy.ROSInterruptException: pass