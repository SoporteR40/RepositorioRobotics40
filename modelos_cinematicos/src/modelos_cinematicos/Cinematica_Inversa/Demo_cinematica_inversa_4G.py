#! /usr/bin/env python

import rospy

# Computing 
from math import acos,atan,sqrt,pi 
from numpy import array

# Message 
from std_msgs.msg import Float64
from openbot_inverse_kinematic.msg import angle

# Server
from dynamic_reconfigure.server import Server


# Configuration 
from modelos_cinematicos.cfg import IK_4GConfig as Ik_4g
# TF
import tf2_ros
import tf2_msgs.msg

# CLass
from modelos_cinematicos.Cinematica_Inversa.inverse_kinematic_computing import Inverse_kinematic_compute
from package_dynamixel.mover_openbot_dynamixel import mover_Dynamixel

class InverseKinematics_4dof(object):
	
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

  			
	def move_openbot_toposition(self,q1,q2,q3,q4):
		""" Move the robot to the position (x,y,z,qy) with the angle we compute"""
		# Command to move the Robot
		angle_joints={"arm1":-q2,
					"arm2":-q3,
					"arm3":0,
					"arm4":-q4,
					"clamp":0,
					"base":q1}
		angle_result=angle()
		angle_result.data=[q1,q2,q3,q4]
		self.pub_angle.publish(angle_result)
		rospy.loginfo('Moving openbot to the position (x,y,z,qy) ...')
		self.move_openbot(angle_joints)

	def move_dynamic_position(self):
		""" Move the robot on a position (x,y,z,qy) with dynamic reconfigure"""
		#command to launch dynamic reconfigure : rosrun rqt_gui rqt_gui -s reconfigure
		
		def callback_position(Ik_4g, level):
			rospy.loginfo("""Reconfigure Request: {Position_X}, {Position_Y}, {Position_Z}, {Angle_qy}, {LOWER_elbow}""".format(**Ik_4g))
			qy=float("{Angle_qy}".format(**Ik_4g))/180 *pi   #Conversion in radian

			if "{Ready}".format(**Ik_4g)=="True":  
				if "{LOWER_elbow}".format(**Ik_4g)=="True": # Check the configuration lower or upper elbow
					tab=Inverse_kinematic_compute.Calcul_angle4GDL_ab(float("{Position_X}".format(**Ik_4g)),float("{Position_Y}".format(**Ik_4g)),float("{Position_Z}".format(**Ik_4g)),qy)
				else:
					tab=Inverse_kinematic_compute.Calcul_angle4GDL_arr(float("{Position_X}".format(**Ik_4g)),float("{Position_Y}".format(**Ik_4g)),float("{Position_Z}".format(**Ik_4g)),qy)
				self.move_openbot_toposition(tab[0],tab[1],tab[2],tab[3]) #move the robot to the position chosen 
				rospy.sleep(2)
				tfBuffer = tf2_ros.Buffer() # To know the position of the effector
				listener = tf2_ros.TransformListener(tfBuffer)
				trans = tfBuffer.lookup_transform('world', 'effector_link_tf', rospy.Time(),rospy.Duration(10))			
				rospy.loginfo("The position in x is : %.2f",-(trans.transform.translation.x-0.005)) # Print the position of the effector
				rospy.loginfo("The position in y is : %.2f",-(trans.transform.translation.y+0.003))
				rospy.loginfo("The position in z is : %.2f",trans.transform.translation.z-0.16)
				if float("{Position_X}".format(**Ik_4g))<0:
					rospy.loginfo("The angle of qy is : %.2f",-(pi/2-tab[2]-tab[1]-tab[3]-pi)*180/pi)
				else:
					rospy.loginfo("The angle of qy is : %.2f",pi/2-tab[2]-tab[1]-tab[3]*180/pi)
				
				rospy.loginfo("")

				if "{Dynamixel}".format(**Ik_4g)=="True":
						print("Moving Dynamixel....") 
						dynamixel= mover_Dynamixel()
						dynamixel.main()
						if float("{Position_X}".format(**Ik_4g)) > 0:
							q1=-tab[1]+0.17
							q2=-tab[2]+0.05
						if float("{Position_X}".format(**Ik_4g)) < 0:
							q1=-tab[1]-0.17
							q2=-tab[2]-0.05
						dynamixel.movimiento_inicial(-tab[0], -q1 ,-q2 ,'no',tab[3],'no')
						#dynamixel.movimiento_inicial(tab[0], tab[1] , tab[2],'no',tab[3],'no')
			else:
				pass
			return Ik_4g
			
		srv = Server(Ik_4g, callback_position)
		rospy.spin()	



if __name__=="__main__":
	openbot_object=InverseKinematics_4dof()
	try:
		openbot_object.move_init_pos()
		openbot_object.move_dynamic_position()   # Starting the rqt_reconfigure	
	except rospy.ROSInterruptException:
		pass
