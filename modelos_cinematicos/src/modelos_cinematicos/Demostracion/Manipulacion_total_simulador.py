#! /usr/bin/env python

import rospy

# Computing 
from math import acos,atan,sqrt,pi 
from numpy import array

# Message 
from std_msgs.msg import Float64

# Server
from dynamic_reconfigure.server import Server

# Configuration 
from modelos_cinematicos.cfg import Angle_jointConfig as config_angle


class Demostracion(object):
	
	def __init__(self):
		"""Initialisation of the object"""
		# Initilisation of the node 
		rospy.init_node('node_move_class')
		rospy.loginfo('Initialising openbot...')
		# Initalisation of the publisher 
		self.pub_arm1 = rospy.Publisher('/openbot_v1/arm1_arm2_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm2 = rospy.Publisher('/openbot_v1/arm2_arm3_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm3 = rospy.Publisher('/openbot_v1/arm3_arm4_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm4 = rospy.Publisher('/openbot_v1/arm4_clamp1_joint_position_controller/command', Float64, queue_size=1)
		self.pub_clamp = rospy.Publisher('/openbot_v1/clamp1_clamp2_joint_position_controller/command', Float64, queue_size=1)
		self.pub_base = rospy.Publisher('/openbot_v1/base_arm1_joint_position_controller/command', Float64, queue_size=1)
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
		# The robot goes to the init position during 3 sec
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

	def move_dynamic_angle(self):
		""" Move the robot by the angle with dynamic reconfigure"""

		#command to launch dynamic reconfigure : rosrun rqt_gui rqt_gui -s reconfigure
		def callback_angle(config_angle, level):
			rospy.loginfo("""Reconfigure Request: {base_arm1_joint}, {arm1_arm2_joint}, {arm2_arm3_joint}, {arm3_arm4_joint}, {arm4_clamp1_joint}, {clamp_joint}""".format(**config_angle))
			angle_joints={"arm1":float("{arm1_arm2_joint}".format(**config_angle)),
					"arm2":float("{arm2_arm3_joint}".format(**config_angle)),
					"arm3":float("{arm3_arm4_joint}".format(**config_angle)),
					"arm4":float("{arm4_clamp1_joint}".format(**config_angle)),
					"clamp":float("{clamp_joint}".format(**config_angle)),
					"base": float("{base_arm1_joint}".format(**config_angle))}
			self.move_openbot(angle_joints)
			return config_angle
		srv = Server(config_angle, callback_angle)
		rospy.spin()


if __name__=="__main__":
	openbot_object=Demostracion()
	try:
		openbot_object.move_init_pos()
		openbot_object.move_dynamic_angle()			
	except rospy.ROSInterruptException:
		pass
