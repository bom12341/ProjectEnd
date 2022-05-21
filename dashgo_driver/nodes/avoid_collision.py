#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
import message_filters

class AvoidCollisionNode():

	def __init__(self, name="avoid_collision_cpe"):
		'''
		'''

		#	Create node name avoid_collision_cpe
		rospy.init_node('avoid_collision_cpe', anonymous=True)

		#	Subscribe 4 sonar sensors
		sonar0_sub = message_filters.Subscriber('sonar0', Range)
		sonar1_sub = message_filters.Subscriber('sonar1', Range)
		sonar2_sub = message_filters.Subscriber('sonar2', Range)
		sonar3_sub = message_filters.Subscriber('sonar3', Range)

		#	Sync 4 sonar message
		self.subscribed_topic = message_filters.TimeSynchronizer([sonar0_sub, sonar1_sub, sonar2_sub, sonar3_sub], 10)

		#	Register callback
		self.subscribed_topic.registerCallback(self.subscribe_cb)

		#	Create publisher topic to send twist msg to robot
		self.publisher = rospy.Publisher('EAI_Collision_control', Twist, queue_size=10)

	def subscribe_cb(self, range0, range1, range2, range3 ):
		''' Callback function when receive Range
		'''
		
		#	log data
		rospy.loginfo('{}: {} {} {} {}'.format(rospy.get_caller_id(), range0.range, range1.range, range2.range, range3.range))

		#	create list of sensor range
		sensorRangeList = [ range0.range, range1.range, range2.range, range3.range ]

		#	Create twist msg
		twist_msg = Twist()

		#	check if any sensor less than 0.2
		#	TODO: Adjust this value
		if ( any( sensor < 0.2 for sensor in sensorRangeList ) ):

			#	TODO: Adjust twist
			twist_msg.linear.x = 0
			twist_msg.angular.z = 20

		#	Go forward if nothing near
		else:
			twist_msg.linear.x = 20

		#	Publish twist msg
		self.publisher.publish(twist_msg)

if __name__ == '__main__':
	AvoidCollisionNode()
	#	While not terminate => loop
	rospy.spin()

