#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
		/os/requestpath			/drone_yaw
								/vrep/waypoints

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
check_drone_flag = 0

list1 = {}

class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [-0.87,0.56,53.35,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [0.66,-0.96,43.75,0.0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.cmd.plutoIndex = 0

		self.count_one_pub = 1
		self.count_no_of_targets = 3

		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [223*0.06,655*0.008,97*0.3,0]
		# self.Ki = [0,0,0,0]
		# self.Kd = [85*0.06,186*0.008,135*0.3,0]

		# self.Kp = [18,18,40,0]
		# self.Ki = [0,0,0,0]
		# self.Kd = [10,5,36,0]

		self.Kp = [ 7.5 ,  7.6  , 50  ,0]
		self.Ki = [ 0.01, 0.06  , 0   ,0]
		self.Kd = [ 370 ,  385  , 80  ,0]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_values = [0,0,0,0]
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]
		self.error = [0,0,0,0]
		self.errorSum=[0,0,0,0]
		

		#pid parameters




		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.030 # in seconds


		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.alt_pub = rospy.Publisher('/alt_error',Float64, queue_size=10)
		self.pit_pub = rospy.Publisher('/pitch_error',Float64, queue_size=10)
		self.roll_pub = rospy.Publisher('/roll_error',Float64, queue_size=10)
		self.yaw_pub = rospy.Publisher('/yaw_error',Float64, queue_size=10)
		self.zero_line_pub = rospy.Publisher('/zero_line_data' ,Int16, queue_size=10)
		# Publishing to vrep requesting a new path from current position to next target

		self.drone_request_pub = rospy.Publisher('/ros/requestpath' ,Int64, queue_size=10)




		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /vrep/waypoints for traversing of drone along  the computed path
	
		rospy.Subscriber('vrep/waypoints', PoseArray, self.call_ba)	
		# Subscribing to /whycon/poses 

		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)


	#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback functione
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		self.drone_position[3] = 0   #msg.poses[0].orientation.z

		#print("bye")


	def call_ba(self,msg):
		

		self.pid()
		rospy.sleep(1)
		self.setpoint[0] = msg.poses[0].position.x
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.setpoint[1] = msg.poses[0].position.y
		self.setpoint[2] = msg.poses[0].position.z
		self.setpoint[3] = 0         #msg.poses[0].orientation.z
		
		for i in range(0,19):

			self.setpoint[0] = msg.poses[i+1].position.x
			self.setpoint[1] = msg.poses[i+1].position.y
			self.setpoint[2] = msg.poses[i+1].position.z
			self.setpoint[3] = 0
					
			self.pid()
			rospy.sleep(1)
		
		# Printing targets on terminal			

		self.count_one_pub = 1	
		if self.count_no_of_targets == 1 or self.count_no_of_targets == 2 :
			print "Next Target\n"
		else :
			print "All Targets reached"	
		time.sleep(2)

		self.count_no_of_targets = self.count_no_of_targets + 1	

	#----------------------------------------------------------------------------------------------------------------------
	


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		
		now=time.time()
		timechange=self.sample_time
		

		dErr = [0,0,0,0]

		

		out = [0,0,0,0]

		self.error[0] = self.drone_position[0] - self.setpoint[0] 
		self.error[1] = self.drone_position[1] - self.setpoint[1] 
		self.error[2] = self.drone_position[2] - self.setpoint[2] 
		self.error[3] = self.drone_position[3] - self.setpoint[3] 

		self.errorSum[0] += self.error[0]*timechange
		self.errorSum[1] += self.error[1]*timechange
		self.errorSum[2] += self.error[2]*timechange
		self.errorSum[3] += self.error[3]*timechange

		
		dErr[0] = (self.error[0] - self.prev_values[0])/timechange
		dErr[1] = (self.error[1] - self.prev_values[1])/timechange
		dErr[2] = (self.error[2] - self.prev_values[2])/timechange
		dErr[3] = (self.error[3] - self.prev_values[3])/timechange

		out[0] = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.errorSum[0]) + (self.Kd[0] * dErr[0])
		out[1] = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.errorSum[1]) + (self.Kd[1] * dErr[1])
		out[2] = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.errorSum[2]) + (self.Kd[2] * dErr[2])
		out[3] = (self.Kp[3] * self.error[3]) + (self.Ki[3] * self.errorSum[3]) + (self.Kd[3] * dErr[3])

		self.cmd.rcRoll = 1500 + out[1]
		self.cmd.rcPitch = 1500 + out[0]
		self.cmd.rcYaw = 1500 + out[3]
		self.cmd.rcThrottle = 1500 + out[2]

		if(self.cmd.rcPitch>self.max_values[0]):
			self.cmd.rcPitch = self.max_values[0]
		if(self.cmd.rcRoll>self.max_values[1]):
			self.cmd.rcRoll = self.max_values[1]
		if(self.cmd.rcThrottle>self.max_values[2]):
			self.cmd.rcThrottle = self.max_values[2]
		if(self.cmd.rcYaw>self.max_values[3]):
			self.cmd.rcYaw = self.max_values[3]	

		if(self.cmd.rcPitch<self.min_values[0]):
			self.cmd.rcPitch = self.min_values[0]
		if(self.cmd.rcRoll<self.min_values[1]):
			self.cmd.rcRoll = self.min_values[1]
		if(self.cmd.rcThrottle<self.min_values[2]):
			self.cmd.rcThrottle = self.min_values[2]
		if(self.cmd.rcYaw<self.min_values[3]):
			self.cmd.rcYaw = self.min_values[3]		

		self.prev_values[0] = self.error[0]
		self.prev_values[1] = self.error[1]
		self.prev_values[2] = self.error[2]
		self.prev_values[3] = self.error[3]
 		
	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)
		# self.alt_pub.publish(self.error[2])
		# self.yaw_pub.publish(self.error[3])
		# self.roll_pub.publish(self.error[1])
		# self.pit_pub.publish(self.error[0])
		# self.zero_line_pub.publish(0)
		
		
		

if __name__ == '__main__':

	e_drone = Edrone()
	rate=rospy.Rate(1/e_drone.sample_time)
	while not rospy.is_shutdown(): 
		e_drone.pid()
		rate.sleep()
		if abs(e_drone.drone_position[0]-e_drone.setpoint[0] <= 0.3) and abs(e_drone.drone_position[1]-e_drone.setpoint[1] <= 0.3) and abs(e_drone.drone_position[2]-e_drone.setpoint[2] <= 0.3) :
			
			check_drone_flag = 1
			
			if e_drone.count_one_pub == 1:
				
				e_drone.drone_request_pub.publish(1)
				
				time.sleep(1)
				e_drone.count_one_pub = 0
				
		if e_drone.count_no_of_targets >= 4:
			e_drone.disarm()			

