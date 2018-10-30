#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import time

def crabSteer(angle, stspeed, drspeed):
	steer_final_angle = angle
	steer_speed = stspeed
	drive_speed = drspeed
	pub = rospy.Publisher('POSE', Float32MultiArray, queue_size=12)
	rospy.init_node('POSE', anonymous=True)
	r = rospy.Rate(100) #100hz

	msg = Float32MultiArray()

	msg.data = [steer_final_angle, steer_final_angle, steer_final_angle, steer_final_angle,
	 steer_speed, steer_speed, steer_speed, steer_speed,
	 drive_speed, drive_speed, drive_speed, drive_speed];
	
	rospy.loginfo(msg)
	pub.publish(msg)

	'''
	#set steer angle
	for i in [0,1,2,3]:
		msg.data[1] = steer_final_angle;
	#set steer speed
	for i in range(4,8):
		msg.data[i] = steer_speed;
	#set drive
	for i in range(8,12):
		msg.data[i+4] = drive_speed;
		'''

	'''
	while not rospy.is_shutdown():
		rospy.loginfo(msg)
		pub.publish(msg)
		r.sleep()
	'''

if __name__ == '__main__':
	try:
		crabSteer(0.25, 2, 1.7)
		time.sleep(2)
		crabSteer(0.50, 3, 2.2)
		time.sleep(2)
		crabSteer(0.70, 2, 1.7)
	except rospy.ROSInterruptException: pass