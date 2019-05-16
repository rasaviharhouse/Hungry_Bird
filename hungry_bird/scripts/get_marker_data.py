#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}

		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):

		#self.dp0 = [msg.poses[0].position.x,msg.poses[0].position.y,msg.poses[0].position.z]
		#self.dp1 = [msg.poses[1].position.x,msg.poses[1].position.y,msg.poses[1].position.z]
		#self.dp2 = [msg.poses[2].position.x,msg.poses[2].position.y,msg.poses[2].position.z]

		self.whycon_marker = {
			0: [msg.poses[0].position.x,msg.poses[0].position.y,msg.poses[0].position.z],
			1: [msg.poses[1].position.x,msg.poses[1].position.y,msg.poses[1].position.z],
			2: [msg.poses[2].position.x,msg.poses[2].position.y,msg.poses[2].position.z]
		}

		#self.drone_position[0] = msg.poses[0].position.y
	       	#self.drone_position[0] = msg.poses[0].position.z

		#self.msg.poses[0].position.x
		#self.msg.poses[0].position.y
		#self.msg.poses[0].position.z

		#self.msg.poses[1].position.x
		#self.msg.poses[1].position.y 
		#self.msg.poses[1].position.z

		#self.msg.poses[2].position.x
		#self.msg.poses[2].position.y
		#self.msg.poses[2].position.z

	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		#print(msg)
		
		self.aruco_marker = {
			0: [msg.markers[0].pose.pose.orientation.x,msg.markers[0].pose.pose.orientation.y,msg.markers[0].pose.pose.orientation.z,msg.markers[0].pose.pose.orientation.w],
			1: [msg.markers[1].pose.pose.orientation.x,msg.markers[1].pose.pose.orientation.y,msg.markers[1].pose.pose.orientation.z,msg.markers[1].pose.pose.orientation.w],
			2: [msg.markers[2].pose.pose.orientation.x,msg.markers[2].pose.pose.orientation.y,msg.markers[2].pose.pose.orientation.z,msg.markers[2].pose.pose.orientation.w]
		}
		
		#self.msg.markers[0].orientation.x
		#self.msg.markers[0].orientation.y
		#self.msg.markers[0].orientation.z
		#self.msg.markers[0].orientation.w

		#self.msg.markers[1].orientation.x
		#self.msg.markers[1].orientation.y
		#self.msg.markers[1].orientation.z
		#self.msg.markers[1].orientation.w

		#self.msg.markers[2].orientation.x
		#self.msg.markers[2].orientation.y
		#self.msg.markers[2].orientation.z
		#self.msg.markers[2].orientation.w
		
		
		# Printing the detected markers on terminal
		print "\n"
		print "WhyCon_marker",self.whycon_marker
		print "ArUco_marker",self.aruco_marker



if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()
