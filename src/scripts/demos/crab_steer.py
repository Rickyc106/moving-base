#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import time
import sys

prev_theta = float( sys.argv[1] )

def crabSteer(angle, stspeed, drspeed):
	steer_final_angle = angle
	steer_speed = stspeed
	drive_speed = drspeed

	pub = rospy.Publisher('MOTOR_ACT', Float32MultiArray, queue_size=12)
	rospy.init_node("talker", anonymous=True)
	rate = rospy.Rate(100) #100Hz
	msg = Float32MultiArray()

	msg.data = [steer_final_angle, steer_final_angle, steer_final_angle, steer_final_angle,
	 steer_speed, steer_speed, steer_speed, steer_speed,
	 drive_speed, drive_speed, drive_speed, drive_speed];
	
	rospy.loginfo(msg)
	pub.publish(msg)


if __name__ == '__main__':
	try:
		print(prev_theta)
		crabSteer(prev_theta+0.25, 2, 1.4)
		time.sleep(5)
		crabSteer(prev_theta+0.5, 1, 2.4)
		time.sleep(5)
		crabSteer(prev_theta+0.75, 2, 1.4)
		time.sleep(5)
	except rospy.ROSInterruptException: pass