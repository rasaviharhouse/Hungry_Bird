#!/usr/bin/env python


# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Char,Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import roslib
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0,0,32.0,0]			# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [0,0,22.0,0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

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


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		# self.Kp = [round(0,2),round(0,2),round(0,2),round(0,2)]
		# self.Ki = [round(0,2),round(0,2),round(0,2),round(0,2)]
		# self.Kd = [round(0,2),round(0,2),round(0,2),round(0,2)]
		# self.Kp = [180*0.06,600*0.008,130*0.3,0]
		# self.Ki = [0,0,0,0]
		# self.Kd = [85*0.06,185*0.008,110*0.3,0]


		#          pitch, roll, throttle ,yaw
		'''
		self.Kp = [5.2, 5, 42, 0]
 		self.Ki = [0, 0,  0, 0]
		self.Kd = [3.47, 3.2, 60, 0]
		'''

		self.Kp = [ 7.5 ,  7.6  , 50  ,0]
		self.Ki = [ 0.01, 0.06  , 0   ,0]
		self.Kd = [ 370 ,  385  , 80  ,0]

		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_values = [0,0,0,0]
		self.max_values = [1600,1600,1700,1700]
		self.min_values = [1400,1400,1200,1200]
		
		self.error = [0,0,0,0]
		self.errorSum=[0,0,0,0]
		
		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.030 # in seconds

		self.prev_time = time.time()







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		# self.alt_pub = rospy.Publisher('/alt_error',Float64, queue_size=10)
		# self.pit_pub = rospy.Publisher('/pitch_error',Float64, queue_size=10)
		# self.roll_pub = rospy.Publisher('/roll_error',Float64, queue_size=10)
		# self.yaw_pub = rospy.Publisher('/yaw_error',Float64, queue_size=10)
		
		# self.zero_line_pub = rospy.Publisher('/zero_line_data' ,Int16, queue_size=10)








		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		# #-------------------------Add other ROS Subscribers here----------------------------------------------------
		# rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		# rospy.Subscriber('pid_tuning_pitch',PidTune,self.pitch_set_pid)
		# rospy.Subscriber('pid_tuning_roll',PidTune,self.roll_set_pid)
		# rospy.Subscriber('pid_tuning_yaw',PidTune,self.yaw_set_pid)
		# rospy.Subscriber('drone_yaw',Float64,self.yaw_cb)


		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(.1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(.1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = round((msg.poses[0].position.x),2)
		self.drone_position[1] = round((msg.poses[0].position.y),2)
		self.drone_position[2] = round((msg.poses[0].position.z),2)
		self.drone_position[3] = round((msg.poses[0].orientation.z),2)
		#self.setpoint = [self.drone_position[0],self.drone_position[1],26.0,0.0]



		
		#---------------------------------------------------------------------------------------------------------------


	# # Callback function for /pid_tuning_altitude
	# # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	# def altitude_set_pid(self,alt):
	# 	self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
	# 	self.Ki[2] = alt.Ki * 0.008
	# 	self.Kd[2] = alt.Kd * 0.3
	# #----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	# def pitch_set_pid(self,pitch):
	# 	self.Kp[0] = pitch.Kp * 0.06# This is just for an example. You can change the fraction value accordingly
	# 	self.Ki[0] = pitch.Ki * 0.008
	# 	self.Kd[0] = pitch.Kd * 0.3
	# def roll_set_pid(self,roll):
	# 	self.Kp[1] = roll.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
	# 	self.Ki[1] = roll.Ki * 0.008
	# 	self.Kd[1] = roll.Kd * 0.03
	# def yaw_set_pid(self,yaw):
	# 	self.Kp[3] = yaw.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
	# 	self.Ki[3] = yaw.Ki * 0.008
	# 	self.Kd[3] = yaw.Kd * 0.3
	# def yaw_cb(self,alt):
	# 	pass


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
		#if(now>self.sample_time):


		# variable that holds the time lapse
		dt = now - self.prev_time
			
		
		timechange=self.sample_time

		# if time lapse less than the sample time, then exit pid function
		if (dt< self.sample_time):
			return

		self.prev_time = now

		dErr = [0,0,0,0]
		out = [0,0,0,0]

		self.error[0] = self.drone_position[0] - self.setpoint[0] 
		self.error[1] = self.drone_position[1] - self.setpoint[1] 
		self.error[2] = self.drone_position[2] - self.setpoint[2] 
		self.error[3] = self.drone_position[3] - self.setpoint[3] 

		self.errorSum[0] += self.error[0]
		self.errorSum[1] += self.error[1]
		self.errorSum[2] += self.error[2]
		self.errorSum[3] += self.error[3]

		dErr[0] = (self.error[0] - self.prev_values[0])
		dErr[1] = (self.error[1] - self.prev_values[1])
		dErr[2] = (self.error[2] - self.prev_values[2])
		dErr[3] = (self.error[3] - self.prev_values[3])

		out[0] = (self.Kp[0] * self.error[0]) + (self.Ki[0] * self.errorSum[0]) + (self.Kd[0] * dErr[0])
		out[1] = (self.Kp[1] * self.error[1]) + (self.Ki[1] * self.errorSum[1]) + (self.Kd[1] * dErr[1])
		out[2] = (self.Kp[2] * self.error[2]) + (self.Ki[2] * self.errorSum[2]) + (self.Kd[2] * dErr[2])
		out[3] = 0 # (self.Kp[3] * self.error[3]) + (self.Ki[3] * self.errorSum[3]) + (self.Kd[3] * dErr[3])

		self.cmd.rcPitch = 1500 + out[0]
		self.cmd.rcRoll = 1500 + out[1]
		self.cmd.rcThrottle = 1500 + out[2]
		self.cmd.rcYaw = 1500 + out[3]

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

		# if(self.error[1]>-0.75 and self.error[1]<0.75):self.drone_position[2]

		

		if(self.error[0]>-0.75 and self.error[0]<0.75):
			print("X-XX-X-X-X-X-X-X-X-X-X-X-X\n")
		if(self.error[1]>-0.75 and self.error[1]<0.75):
			print("Y-YY-Y-Y-Y-Y-Y-Y-Y-Y-Y-Y-Y\n")
		if(self.error[2]>-1.0 and self.error[2]<1.0):
			print("Z-ZZ-Z-Z-Z-Z-Z-Z-Z-Z-Z-Z-Z", self.drone_position[2],"\n")

        
		self.prev_values[0] = self.error[0]
		self.prev_values[1] = self.error[1]
		self.prev_values[2] = self.error[2]
		self.prev_values[3] = self.error[3]

		print(self.drone_position[0],self.drone_position[1],self.drone_position[2])



		#self.last_time = now
		
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
		#rate.sleep()
