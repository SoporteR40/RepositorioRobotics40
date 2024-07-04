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

# TF
import tf2_ros
# CLass
from mover_openbot_dynamixel import mover_Dynamixel

#Marker for RVIZ
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

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
		#Publisher de rviz
		self.pub_marker= rospy.Publisher('visualization_marker_array', MarkerArray,queue_size=10)
		rospy.on_shutdown(self.shutdownhook)
	
	def shutdownhook(self):
		# works better than the rospy.is_shutdown()
		self.ctrl_c = True

	def Create_markers(header="effector_link_tf"):
			"""Creat the marker for RVIZ"""
			markerArray = MarkerArray()
			marker=Marker()
			marker.header.frame_id = header
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.scale.x = 0.01
			marker.scale.y = 0.01
			marker.scale.z = 0.01
			marker.color.a = 0.6
			marker.color.r = 0.4
			marker.color.g = 0.1
			marker.color.b = 0.5
			marker.pose.orientation.w = 1.0
			marker.pose.position.x = 0
			marker.pose.position.y = 0
			marker.pose.position.z = 0
			return markerArray,marker


	def move_init_pos(self):
		Dynamixel_state="True"
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
			self.move_openbot(angle_joints,Dynamixel_state)
			data=data+1
			rate.sleep()
		rospy.loginfo('Ending of the init position')
		
	def move_openbot(self, angle_joints,Dynamixel_state):
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
		if Dynamixel_state == "True":
			Dynamixel=mover_Dynamixel()
			Dynamixel.movimiento_inicial(-angle_joints["base"],-angle_joints["arm1"],-angle_joints["arm2"],-angle_joints["arm3"],-angle_joints["arm4"],-angle_joints["clamp"])
		

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
			rospy.loginfo("Sum of the angle : %.2f",pi/2-float("{arm1_arm2_joint}".format(**config_angle))+float("{arm2_arm3_joint}".format(**config_angle))+float("{arm4_clamp1_joint}".format(**config_angle)))
			
			if "{Simulacion}".format(**config_angle)=="True":
				Dynamixel_state="{Dynamixel}".format(**config_angle)
				self.move_openbot(angle_joints,Dynamixel_state)
				rospy.sleep(1)
				tfBuffer = tf2_ros.Buffer()    # To know the position of the effector
				listener = tf2_ros.TransformListener(tfBuffer) 
				trans = tfBuffer.lookup_transform('world', 'effector_link_tf', rospy.Time(),rospy.Duration(10))			
				rospy.loginfo("The position in x is : %.2f",-(trans.transform.translation.x)) # Print the position of the effector
				rospy.loginfo("The position in z is : %.2f",trans.transform.translation.z-0.16)
				id=0
				markerArray,marker=Demostracion.Create_markers()
				markerArray.markers.append(marker)
				for m in markerArray.markers:
					m.id = id
					id += 1
				self.pub_marker.publish(markerArray)

			if "{Posicion_inicial}".format(**config_angle)=="True":
				Dynamixel_state="{Dynamixel}".format(**config_angle)
				self.move_init_pos()
			return config_angle
		srv = Server(config_angle, callback_angle)
		rospy.spin()
	
	


if __name__=="__main__":
	openbot_object=Demostracion()
	dynamixel=mover_Dynamixel()
	dynamixel.main()
	try:
		openbot_object.move_dynamic_angle()		
	except rospy.ROSInterruptException:
		pass
