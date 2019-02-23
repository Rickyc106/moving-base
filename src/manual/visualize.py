#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Float32MultiArray
from numpy import *

class Visualize:
	def __init__(self):
		self.resultant_array = Float32MultiArray()
		self.resultant_array.data = zeros(4)

	def array_cb(self, msg):
		for i in range(4):
			self.resultant_array.data[i] = msg.data[i]

	def main(self):
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():
			pose_array = PoseArray()
			pose_array.header.frame_id = 'map'
			pose_array.header.stamp = rospy.Time.now()

			resultant = self.resultant_array.data

			quat0 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[0]))
			quat1 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[1]))
			quat2 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[2]))
			quat3 = Quaternion(*tf.transformations.quaternion_from_euler(0,0,resultant[3]))

			point0 = Point(-1,1,0)
			point1 = Point(-1,-1,0)
			point2 = Point(1,-1,0)
			point3 = Point(1,1,0)

			pose_array.poses.append(Pose(point0, quat0))
			pose_array.poses.append(Pose(point1, quat1))
			pose_array.poses.append(Pose(point2, quat2))
			pose_array.poses.append(Pose(point3, quat3))

			arrow_pub.publish(pose_array)
			rate.sleep()

if __name__ == '__main__':
	try:
		print "Visualize is Alive!"
		rospy.init_node('visualize', anonymous=True)
		arrow_pub = rospy.Publisher('resultant_pose_array', PoseArray, queue_size=10)
		
		V = Visualize()
		rospy.Subscriber('resultant_angle', Float32MultiArray, V.array_cb)
		V.main()
	except rospy.ROSInterruptException:
		pass
