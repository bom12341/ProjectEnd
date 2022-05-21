#!/usr/bin/env python

import rospy
import roslib
# roslib.load_manifest('learning_tf')
import math
import tf
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Vector3
import message_filters
from tf import transformations as t

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

		#	Create transform listener
		self.tf_listener = tf.TransformListener()

		#	Wait for first transform	
		self.tf_listener.waitForTransform("base_footprint", "sonar3", rospy.Time(0), rospy.Duration(1.0))

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

			#	Not work yet
			# self.turnOdom(1.57, True)

		#	Go forward if nothing near
		else:
			twist_msg.linear.x = 20

		#	Publish twist msg
		self.publisher.publish(twist_msg)

	def turnOdom(self, radians, isClockwise):
		'''
		'''

		while(radians < 0):
			radians += 2*math.pi
		
		while(radians > 2*math.pi):
			radians -= 2*math.pi

		#	Wait for first transform	
		self.tf_listener.waitForTransform("base_footprint", "sonar3", rospy.Time(0), rospy.Duration(1.0))

		#	Record start transform
		(trans, rot) = self.tf_listener.lookupTransform("base_footprint", "sonar3", rospy.Time(0))

		start_transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))

		twist_msg = Twist()

		#	the command will be to turn at 0.75 rad/s
		twist_msg.linear.x = twist_msg.linear.y = 0.0;
		twist_msg.angular.z = 0.75;
		if (isClockwise):
			twist_msg.angular.z = -twist_msg.angular.z;
		
		#	the axis we want to be rotating by
		desired_turn_axis = Vector3(0,0,1)
		if ( not isClockwise ):
			desired_turn_axis = -desired_turn_axis;
		
		rate = rospy.Rate(10.0)
		done = False
		current_transform = None
		while( not done ):
			
			#	send the drive command
			self.publisher.publish(twist_msg);
			rate.sleep();
			#	get the current transform
			try:
				(trans, rot) = self.tf_listener.lookupTransform("base_footprint", "sonar3", rospy.Time(0))

				current_transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))

			except tf.LookupException as e:
				rospy.logerr("{}".format(e.what()))
				break

			relative_transform = t.inverse_matrix(start_transform) * current_transform
			actual_turn_axis = relative_transform[:3]
			angle_turned = 0.5 * math.cos(relative_transform[-1].sum())
			if ( math.fabs(angle_turned) < 1.0e-2):
				continue

			if ( Vector3(actual_turn_axis).dot( desired_turn_axis ) < 0 ):
				angle_turned = 2 * math.pi - angle_turned;

			if (angle_turned > radians):
				done = True
		
		if (done):
			return True
		return False

if __name__ == '__main__':
	AvoidCollisionNode()
	#	While not terminate => loop
	rospy.spin()

