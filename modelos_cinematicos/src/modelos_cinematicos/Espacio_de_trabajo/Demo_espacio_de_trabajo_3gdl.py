#! /usr/bin/env python
from symbol import parameters
import rospy
import os
# Computing 
from math import acos,atan,sqrt,pi,cos,sin
import numpy as np
# Message 
from std_msgs.msg import Float64
from std_msgs.msg import String
# Server
from dynamic_reconfigure.server import Server
# TF
import tf2_ros
# Configuration 

from modelos_cinematicos.cfg import workspace_3gConfig as workspace_3g
# CLass
from modelos_cinematicos.Cinematica_Inversa.cinematica_inversa_calculos import calculos_cinematica_inversa
from package_dynamixel.mover_openbot_dynamixel import mover_Dynamixel
from modelos_cinematicos.Espacio_de_trabajo.Espacio_trabajo_openbot import Espacio_de_trabajo
from modelos_cinematicos.Espacio_de_trabajo.Graficas_Espacio_trabajo_openbot import Graficas_Espacio_de_trabajo

#Marker for RVIZ
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class workspace_3gdl(object):
	
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
		#configuraciones
		self.pub_configuracion= rospy.Publisher('/configuration_rqt', String,queue_size=1)
		rospy.on_shutdown(self.shutdownhook)

	## Seccion de rviz

	def Create_marker(header="effector_link_tf"):
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

	def Delete_marker(self):
			"""Clean the marker on RVIZ"""
			markerArray = MarkerArray()
			marker=Marker()
			marker.action = marker.DELETEALL
			markerArray.markers.append(marker)
			self.pub_marker.publish(markerArray)


	def Rviz_workspace_completo(self,coordenadas):
		self.Delete_marker()
		markerArray = MarkerArray()
		marker=Marker()
		marker.header.frame_id = "arm2_link_1"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = -0.02
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.scale.z = 0.01
		marker.color.a = 0.6
		marker.color.r = 0.4
		marker.color.g = 0.1
		marker.color.b = 0.5
		marker.pose.orientation.w = 1.0
		marker.pose.position.y = 0
		marker.lifetime=rospy.Duration(1000)
		N=0
		rate=rospy.Rate(1)
		id=0
		while not rospy.is_shutdown() and N<5:
			for par in coordenadas:
				for i in range(0,len(par[0])):
					marker.pose.position.x = -par[0][i]
					marker.pose.position.z = par[1][i]
					marker.id=id
					#print(markerArray.markers)
					markerArray.markers.append(marker)
					self.pub_marker.publish(markerArray)
					id += 1
			N=N+1
			rate.sleep()



	def Openbot_trazo_espacio_de_trabajo_abajo(self):
		n=10
		#Primer limite
		limite_1=np.linspace(1.57,-1.57,num=n,endpoint=True)

		for i in limite_1:
			self.mover_demo(i, 0,0)

		#Segundo limite
		limite_2=np.linspace(0,1.57,num=n,endpoint=True)
		for i in limite_2:
			self.mover_demo(-1.57,0, i )

		#Tercer limite
		limite_3=np.linspace(0,2.09,num=n,endpoint=True)
		for i in limite_3:
			self.mover_demo(-1.57,i ,1.57 )   
		#Cuarto Limite
		limite_4=np.linspace(-1.57,-1.335,num=n,endpoint=True)
		for i in limite_4:
			self.mover_demo(i,2.09 ,1.57 )     

	def Openbot_trazo_espacio_de_trabajo_arriba(self):
		n=10
		#Primer limite
		limite_1=np.linspace(-1.57,1.57,num=n,endpoint=True)

		for i in limite_1:
			self.mover_demo(i, 0,0)

		#Segundo limite
		limite_2=np.linspace(0,-1.57,num=n,endpoint=True)
		for i in limite_2:
			self.mover_demo(1.57,0, i )

		#Tercer limite
		limite_3=np.linspace(0,-2.09,num=n,endpoint=True)
		for i in limite_3:
			self.mover_demo(1.57,i ,-1.57 )   
		#Cuarto Limite
		limite_4=np.linspace(1.57,1.335,num=n,endpoint=True)
		for i in limite_4:
			self.mover_demo(i,-2.09 ,-1.57 )     


	def clearConsole(self):
		command = 'clear'
		if os.name in ('nt', 'dos'):  # If Machine is running on Windows, use cls
			command = 'cls'
		os.system(command)
	
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

  			
	def move_openbot_toposition(self,q1,q2,q3,x):
		""" Move the robot to the position (x,z,qy) with the angle we compute"""
		# Command to move the Robot
		angle_joints={"arm1":-q1,
					"arm2":-q2,
					"arm3":0,
					"arm4":-q3,
					"clamp":0,
					"base":0}
		rospy.loginfo('Moving openbot to the position (x,z,qy) ...')
		self.move_openbot(angle_joints)
		rospy.sleep(1)
		tfBuffer = tf2_ros.Buffer()    # To know the position of the effector
		listener = tf2_ros.TransformListener(tfBuffer) 
		trans = tfBuffer.lookup_transform('world', 'effector_link_tf', rospy.Time(),rospy.Duration(10))			
		rospy.loginfo("The position in x is : %.2f",-(trans.transform.translation.x)) # Print the position of the effector
		rospy.loginfo("The position in z is : %.2f",trans.transform.translation.z-0.16)
		if x<0:
				rospy.loginfo("The angle of qy is : %.2f",((pi/2-q1-q2-q3)*180/pi))
		else:
				rospy.loginfo("The angle of qy is : %.2f", (pi/2-q1-q2-q3)*180/pi)
	
	def mover_demo(self,q1,q2,q3):
		""" Move the robot to the position (x,z)"""
		# Command to move the Robot
		angle_joints={"arm1":q1,
					"arm2":q2,
					"arm3":0,
					"arm4":q3,
					"clamp":0,
					"base":0}
		rospy.loginfo('Moving openbot to the position (x,z) ...')
		print("Angle_joints",angle_joints)
		self.move_openbot(angle_joints)
		rospy.sleep(1)
		tfBuffer = tf2_ros.Buffer()    # To know the position of the effector
		listener = tf2_ros.TransformListener(tfBuffer) 
		trans = tfBuffer.lookup_transform('world', 'effector_link_tf', rospy.Time(),rospy.Duration(10))			
		rospy.loginfo("The position in x is : %.2f",-(trans.transform.translation.x)) # Print the position of the effector
		rospy.loginfo("The position in z is : %.2f",trans.transform.translation.z-0.16)


	def calculos(self,parameters):
			print(parameters)
			x=parameters[0]
			z=parameters[1]
			qy=parameters[2]
			L1=parameters[3]
			L2=parameters[4]
			L3=parameters[5]
			codo=parameters[6]

			try:
				if codo == "True":
					result=calculos_cinematica_inversa.cal_3GDL_aba(x,z,qy,L1,L2,L3)
				if codo ==  "False":
					result=calculos_cinematica_inversa.cal_3GDL_arri(x,z,qy,L1,L2,L3)
				print("Result: ",result)
			except ValueError :
				print("Error Matematico, Punto por fuera del espacio de trabajo")
				result="Error"
			return result


	def move_dynamic_position(self):
		""" Move the robot on a position (x,z) with dynamic reconfigure"""
		#command to launch dynamic reconfigure : rosrun rqt_gui rqt_gui -s reconfigure
			
		def callback_ready(workspace_3g, level):
			openbot_object=workspace_3gdl()
			dynamixel=mover_Dynamixel()
			L1=0.094
			L2=0.112
			L3=0.15

			if "{Workspace}".format(**workspace_3g)=="True" and "{LOWER_elbow}".format(**workspace_3g)=="True":

				self.move_init_pos()
				workspace=Espacio_de_trabajo()
				#Traigo las coordenadas
				coordenadas=workspace.workspace_3g_abajo()
				self.Rviz_workspace_completo(coordenadas)
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("1")
				print("1")
				rate.sleep()

			if "{Workspace}".format(**workspace_3g)=="True" and "{LOWER_elbow}".format(**workspace_3g)=="False":
				self.move_init_pos()
				workspace=Espacio_de_trabajo()
				coordenadas=workspace.workspace_3g_arriba()
				self.Rviz_workspace_completo(coordenadas)
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("1")
				print("1")
				rate.sleep()

			if "{Borrar_workspace}".format(**workspace_3g)=="True":
				self.Delete_marker()
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("2")
				print("2")
				rate.sleep()

			if "{Grafica_Python}".format(**workspace_3g)=="True":
				codo="{LOWER_elbow}".format(**workspace_3g)
				Graficas=Graficas_Espacio_de_trabajo()
				if codo=="True":
					Graficas.grafica_workspace_3g_abajo()
				elif codo == "False":
					Graficas.grafica_workspace_3g_arriba()

			if "{Workspace}".format(**workspace_3g)=="False" and "{Demo}".format(**workspace_3g)=="True" and "{LOWER_elbow}".format(**workspace_3g)=="True"and "{Ready}".format(**workspace_3g)=="False":
				self.Openbot_trazo_espacio_de_trabajo_abajo()

			if "{Workspace}".format(**workspace_3g)=="False" and "{Demo}".format(**workspace_3g)=="True"and "{LOWER_elbow}".format(**workspace_3g)=="False" and "{Ready}".format(**workspace_3g)=="False":
				self.Openbot_trazo_espacio_de_trabajo_arriba()

			if "{Ready}".format(**workspace_3g)=="True":  
					parameters=[float("{Position_X}".format(**workspace_3g)),float("{Position_Z}".format(**workspace_3g)),float("{Angle_qy}".format(**workspace_3g)),L1,L2,L3,"{LOWER_elbow}".format(**workspace_3g)]
					print("Parametros: ",parameters)
					result=openbot_object.calculos(parameters) #Calculos de Cinematica Inversa
					if result == "Error":
						print("El robot no se moverà")
					else:	
						openbot_object.move_openbot_toposition(result[0], result[1],result[2],parameters[0] ) #Mover Simulacion
						id=0
						markerArray,marker=workspace_3gdl.Create_marker()
						markerArray.markers.append(marker)
						for m in markerArray.markers:
							m.id = id
							id += 1
						self.pub_marker.publish(markerArray)
						
						if "{Dynamixel}".format(**workspace_3g)=="True":
							print("Moving Dynamixel....") 
							dynamixel.main()
							if float("{Position_X}".format(**workspace_3g)) > 0 and float("{Position_X}".format(**workspace_3g))  >= 0.15:
								q1=-result[0]+0.17
								q2=-result[1]+0.05
							elif float("{Position_X}".format(**workspace_3g)) < 0 and float("{Position_X}".format(**workspace_3g)) <= -0.15:
								q1=-result[0]-0.17
								q2=-result[1]-0.05
							else:
								q1=-result[0]
								q2=-result[1]
							dynamixel.movimiento_inicial('no', -q1 ,-q2 ,'no',result[2],'no')
			else:
				pass
			return workspace_3g

		srv = Server(workspace_3g, callback_ready)
		rospy.spin()


if __name__=="__main__":
	openbot_object=workspace_3gdl()
	try:
		openbot_object.move_init_pos()     
		openbot_object.move_dynamic_position()   		
	except rospy.ROSInterruptException:
		pass

