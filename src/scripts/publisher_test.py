#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray

def talker():
	pub = rospy.Publisher('POSE', Float32MultiArray, queue_size=10)
	rospy.init_node('POSE', anonymous=True)
	r = rospy.Rate(10) #10hz
	msg = Float32MultiArray()
	msg.data = [1,1,1,1,0.2,0.2,0.2,0.2]
	while not rospy.is_shutdown():
		rospy.loginfo(msg)
		pub.publish(msg)
		r.sleep()
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass