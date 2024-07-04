#! /usr/bin/env python

from numpy.lib.function_base import delete
import rospy

# Computing
from math import acos,atan,sqrt,pi,sin 
import numpy as np

#Graphics

import matplotlib

from rospy.timer import Rate
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import tkinter

# Message 
from std_msgs.msg import Float64
from openbot_jacobian.msg import angle

# Server
from dynamic_reconfigure.server import Server


# Configuration 
#from openbot_jacobian.cfg import Square2DConfig as config_square
#from openbot_jacobian.cfg import Circle2DConfig as config_circle
#from openbot_jacobian.cfg import Triangle2DConfig as config_triangle

# TF
import tf2_ros
import tf2_msgs.msg

# CLass
from calculos_cinematica_inversa import calculos_cinematica_inversa
from splinecub_dynamixel import splinecub_2
from splinecub_puntos import splinecub
#from openbot_jacobian.jacobian_computing import Jacobian_compute

#Marker for RVIZ
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Move_Object_3dof(object):
	
	def __init__(self):
		"""Initialisation of the object"""
		# Initilisation of the node 
		rospy.init_node('node_move_object')
		rospy.loginfo('Initialising openbot...')
		# Initalisation of the publisher 
		self.pub_arm1 = rospy.Publisher('/openbot_v1/arm1_arm2_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm2 = rospy.Publisher('/openbot_v1/arm2_arm3_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm3 = rospy.Publisher('/openbot_v1/arm3_arm4_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm4 = rospy.Publisher('/openbot_v1/arm4_clamp1_joint_position_controller/command', Float64, queue_size=1)
		self.pub_clamp = rospy.Publisher('/openbot_v1/clamp1_clamp2_joint_position_controller/command', Float64, queue_size=1)
		self.pub_base = rospy.Publisher('/openbot_v1/base_arm1_joint_position_controller/command', Float64, queue_size=1)
		self.pub_angle = rospy.Publisher('/angle_joint',angle,queue_size=10)
		rospy.on_shutdown(self.shutdownhook)
	
	def shutdownhook(self):
		# works better than the rospy.is_shutdown()
		self.ctrl_c = True

	def move_init_pos(self):
		"""Move the robot to the init position during 3 iteration"""
		rate=rospy.Rate(1)

		rospy.loginfo('Moving openbot to the init position...')
		data=0
		angle_joints={"arm1":0,
				"arm2":0,
				"arm3":0,
				"arm4":0,
				"clamp":0,
				"base":0}
		
		while data!=3:
			self.move_openbot(angle_joints)
			data=data+1
			rate.sleep()
		rospy.loginfo('Ending of the init position')
		
	def move_openbot(self, angle_joints):
		""" Move the robot to the angle we want"""
		
		# Initialisation of command
		cmd_arm1=Float64()
		cmd_arm2=Float64()
		cmd_arm3=Float64()
		cmd_arm4=Float64()
		cmd_clamp=Float64()
		cmd_base=Float64()

		# Command to move the Robot
		cmd_arm1.data=angle_joints["arm1"]
		cmd_arm2.data=angle_joints["arm2"]
		cmd_arm3.data=angle_joints["arm3"]
		cmd_arm4.data=angle_joints["arm4"]
		cmd_clamp.data=angle_joints["clamp"]
		cmd_base.data=angle_joints["base"]
		rospy.loginfo('Moving openbot...')
		
		# Publication of the command
		self.pub_arm1.publish(cmd_arm1)
		self.pub_arm2.publish(cmd_arm2)
		self.pub_arm3.publish(cmd_arm3)
		self.pub_arm4.publish(cmd_arm4)
		self.pub_clamp.publish(cmd_clamp)
		self.pub_base.publish(cmd_base)

  			
	def move_openbot_toposition(self,q1,q2,q3,q4):
		""" Move the robot to the position (x,z)"""

		# Command to move the Robot
		angle_joints={"arm1":-q1,
					"arm2":-q2,
					"arm3":pi/2,
					"arm4":-q3,
					"clamp":q4,
					"base":0}
		angle_result=angle()
		angle_result.data=[q1,q2,q3,q4]
		self.pub_angle.publish(angle_result)
		rospy.loginfo('Moving openbot to the position (x,z) ...')
		self.move_openbot(angle_joints)
	
	def move_trajectorie_init(self,N=70,Tf=6,x1=0.0,x2=0.35,z1=0.35,z2=0.0,Upper=True,q_clamp=1.5,qy1=1.57,qy2=0.0):
		"""Move the robot with the trajectorie we compute"""
		rate=rospy.Rate(N/Tf)
		if Upper==True:
			Les_q1,Les_q2,Les_q3,q1_simul,q2_simul,q3_simul=Trajectorie_compute.q1_q2_q3_trajectorie_compute_3dof_arr(N,Tf,x1,x2,z1,z2,qy1,qy2) # Compute the value of q1 and q2 and q3 trajectories
		else:
			Les_q1,Les_q2,Les_q3,q1_simul,q2_simul,q3_simul=Trajectorie_compute.q1_q2_q3_trajectorie_compute_3dof_ab(N,Tf,x1,x2,z1,z2,qy1,qy2) # Compute the value of q1 and q2 and q3 trajectories
		d=0
		while not rospy.is_shutdown() and d<=N-1:
			self.move_openbot_toposition(Les_q1[d],Les_q2[d],Les_q3[d],q_clamp)
			rate.sleep()
			d=d+1


	def move_clamp(self,q_clamp):
		"""Move the clamp by publishing only in his topic"""
		cmd_clamp=Float64()
		cmd_clamp.data=q_clamp
		self.pub_clamp.publish(cmd_clamp)

	def Clamp_trajectory(self,N=1,Tf=1,q1_clamp=0.0,q2_clamp=-1):
		"""Compute a trajectory and did it for the clamp"""
		rate=rospy.Rate(N/Tf)
		Les_clamp=Clamp_compute.compute_clamp_list(Tf,q1_clamp,q2_clamp,N)
		d=0
		
		while not rospy.is_shutdown() and d<=N-1:
			self.move_clamp(Les_clamp[d])
			rospy.loginfo(Les_clamp[d])
			rate.sleep()
			d=d+1


	def Close_clamp(self,q3):
		"""Close the clamp without trajectory"""
		cmd_clamp=Float64()
		cmd_clamp.data=q3
		self.pub_clamp.publish(cmd_clamp)
	
	def Rotation_clamp(self,q3):
		"""Close the clamp without trajectory"""
		cmd_arm3=Float64()
		cmd_arm3.data=q3
		self.pub_arm3.publish(cmd_arm3)

	def Open_clamp(self):
		"""Open the clamp to let the object """
		self.Clamp_trajectory(q1_clamp=0.25,q2_clamp=1.5,N=5,Tf=2)

	def Grab_object(self):
		# Part 1 : Go to the position and grab the object
		#self.Open_clamp()
		self.move_trajectorie_init()
		self.move_trajectorie_init(x1=0.35,z1=0.0,x2=0.155,z2=-0.2,qy1=0.0,qy2=-1.2)
		self.Rotation_clamp(pi/2)
		rospy.sleep(1)
		self.Close_clamp(0.25)
		# rospy.sleep(1)
		self.move_trajectorie_init(x1=0.155,z1=-0.2,q_clamp=0.25,qy1=-1.0,qy2=0.0)
		self.move_trajectorie_init(x1=0.35,z1=0.0,x2=0.0,z2=0.35,q_clamp=0.25,qy1=0.0,qy2=1.57)
		# #Part 2 : Go to the positon to let the object
		self.move_trajectorie_init(x2=-0.35,z2=0.0,q_clamp=0.25,qy1=1.57,qy2=0.0)
		self.move_trajectorie_init(x1=-0.35,z1=0.0,x2=-0.155,z2=-0.2,q_clamp=0.25,qy1=0.0,qy2=-1.3)
		self.Open_clamp()
		rospy.sleep(1)
		self.move_trajectorie_init(x1=-0.155,z1=-0.2,x2=-0.35,z2=0.0,qy1=-1.2,qy2=0.0)
		self.Close_clamp(0)
		self.move_trajectorie_init(x1=-0.35,z1=0.0,x2=0.0,z2=0.373,q_clamp=0,qy1=0.0,qy2=1.57)
		self.move_init_pos()



if __name__=="__main__":
	openbot_object=Move_Object_3dof()
	try:
		openbot_object.Grab_object()  # Does the trajectory to grab the object and let the cube on the other side
	except rospy.ROSInterruptException:
		pass