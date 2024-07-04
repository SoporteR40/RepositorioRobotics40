#! /usr/bin/env python
from email.header import Header
from string import punctuation
from symbol import parameters

from requests import head
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
from modelos_cinematicos.cfg import trayectoria_3gConfig as trayectoria_2g
# CLass
from modelos_cinematicos.Cinematica_Inversa.cinematica_inversa_calculos import calculos_cinematica_inversa
from package_dynamixel.dynamixel_trayectoria import Dynamixel_trayectoria
from modelos_cinematicos.Trayectorias.splinecub_puntos import splinecub
from modelos_cinematicos.Espacio_de_trabajo.Espacio_trabajo_openbot import Espacio_de_trabajo
from modelos_cinematicos.Trayectorias.Graficas_Espacio_trabajo_openbot_figuras import Graficas_Espacio_de_trabajo_figuras

#Marker for RVIZ
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point



class trayectorie_2dof(object):
	
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

  			
	def move_openbot_toposition(self,q1,q2):
		""" Move the robot to the position (x,z) with the angle we compute"""
		# Command to move the Robot
		angle_joints={"arm1":-q1,
					"arm2":-q2,
					"arm3":0,
					"arm4":0,
					"clamp":0,
					"base":0}
		rospy.loginfo('Moving openbot to the position (x,z) ...')
		self.move_openbot(angle_joints)
		rospy.sleep(1)
		tfBuffer = tf2_ros.Buffer()    # To know the position of the effector
		listener = tf2_ros.TransformListener(tfBuffer) 
		trans = tfBuffer.lookup_transform('world', 'effector_link_tf', rospy.Time(),rospy.Duration(10))			
		rospy.loginfo("The position in x is : %.2f",-(trans.transform.translation.x)) # Print the position of the effector
		rospy.loginfo("The position in z is : %.2f",trans.transform.translation.z-0.16)
	

	def move_dynamic_position(self):
		""" Move the robot on a position (x,z) with dynamic reconfigure"""
		#command to launch dynamic reconfigure : rosrun rqt_gui rqt_gui -s reconfigure
			
		def callback_ready(trayectoria_2g, level):
			openbot_object=trayectorie_2dof()

			if "{Workspace}".format(**trayectoria_2g)=="True" and "{LOWER_elbow}".format(**trayectoria_2g)=="True":

				self.move_init_pos()
				workspace=Espacio_de_trabajo()
				#Traigo las coordenadas
				coordenadas=workspace.workspace_2g_abajo()
				self.Rviz_workspace_completo(coordenadas)
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("1")
				print("1")
				rate.sleep()

			if "{Workspace}".format(**trayectoria_2g)=="True" and "{LOWER_elbow}".format(**trayectoria_2g)=="False":
				self.move_init_pos()
				workspace=Espacio_de_trabajo()
				coordenadas=workspace.workspace_2g_arriba()
				self.Rviz_workspace_completo(coordenadas)
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("1")
				print("1")
				rate.sleep()

			if "{Borrar_workspace}".format(**trayectoria_2g)=="True":
				self.Delete_marker()
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("2")
				print("2")
				rate.sleep()
			
			if "{Grafica_Python}".format(**trayectoria_2g)=="True":

				codo="{LOWER_elbow}".format(**trayectoria_2g)
				if "{Cuadrado}".format(**trayectoria_2g)=="True" and "{Triangulo}".format(**trayectoria_2g)=="False" and "{Circulo}".format(**trayectoria_2g)=="False":
					figura="1"
				elif "{Cuadrado}".format(**trayectoria_2g)=="False" and "{Triangulo}".format(**trayectoria_2g)=="True"and "{Circulo}".format(**trayectoria_2g)=="False":
					figura="2"
				elif "{Cuadrado}".format(**trayectoria_2g)=="False" and "{Triangulo}".format(**trayectoria_2g)=="False"and "{Circulo}".format(**trayectoria_2g)=="True":
					figura="3"
				else:
					figura="no"
				
				Graficas=Graficas_Espacio_de_trabajo_figuras()

				if codo=="True":
					Graficas.grafica_workspace_2g_abajo_figuras(figura)
				elif codo == "False":
					Graficas.grafica_workspace_2g_arriba_figuras(figura)


			if "{Ready}".format(**trayectoria_2g)=="True" and  "{mov_inicial}".format(**trayectoria_2g)=="False": 
					codo="{LOWER_elbow}".format(**trayectoria_2g)
					
					if "{Cuadrado}".format(**trayectoria_2g)=="True" and "{Triangulo}".format(**trayectoria_2g)=="False" and "{Circulo}".format(**trayectoria_2g)=="False":
						figura="cuadrado"
						if "{Simulacion}".format(**trayectoria_2g)=="True" and "{Dynamixel}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="False":
							openbot_object.trayectoria_simulada(figura,codo)
						if "{Dynamixel}".format(**trayectoria_2g)=="True" and "{Simulacion}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="False":
							openbot_object.trayectoria_real(figura,codo)
						if "{Dynamixel}".format(**trayectoria_2g)=="False" and "{Simulacion}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="True":
							openbot_object.trayectoria_solo_real(figura,codo)

					if "{Cuadrado}".format(**trayectoria_2g)=="False" and "{Triangulo}".format(**trayectoria_2g)=="True"and "{Circulo}".format(**trayectoria_2g)=="False":
						figura="triangulo"
						if "{Simulacion}".format(**trayectoria_2g)=="True" and "{Dynamixel}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="False":
							openbot_object.trayectoria_simulada(figura,codo)
						if "{Dynamixel}".format(**trayectoria_2g)=="True" and "{Simulacion}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="False":
							openbot_object.trayectoria_real(figura,codo)
						if "{Dynamixel}".format(**trayectoria_2g)=="False" and "{Simulacion}".format(**trayectoria_2g)=="False" and"{Solo_Dynamixel}".format(**trayectoria_2g)=="True":
							openbot_object.trayectoria_solo_real(figura,codo)

					if "{Cuadrado}".format(**trayectoria_2g)=="False" and "{Triangulo}".format(**trayectoria_2g)=="False"and "{Circulo}".format(**trayectoria_2g)=="True":
						figura="circulo"
						if "{Simulacion}".format(**trayectoria_2g)=="True" and "{Dynamixel}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="False":
							openbot_object.trayectoria_simulada(figura,codo)
						if "{Dynamixel}".format(**trayectoria_2g)=="True" and "{Simulacion}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="False":
							openbot_object.trayectoria_real(figura,codo)
						if "{Dynamixel}".format(**trayectoria_2g)=="False" and "{Simulacion}".format(**trayectoria_2g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_2g)=="True":
							openbot_object.trayectoria_solo_real(figura,codo)

			elif "{mov_inicial}".format(**trayectoria_2g)=="True" and "{Ready}".format(**trayectoria_2g)=="False" :
				self.move_init_pos()
				Dynamixel= Dynamixel_trayectoria()
				Dynamixel.main()
				Dynamixel.movimiento_inicial(0,0,0,0,0,-0.345)
			else:
				pass
			return trayectoria_2g

		srv = Server(trayectoria_2g, callback_ready)
		rospy.spin()

	def trazo_cuadrado(self,n,tf,codo):
		#Parametros del cuadrado
		if codo =="True":
			corrimientox=-0.01
			corriemientoz=0.25
			base=-0.06
			altura=0.06

		else:
			corrimientox=0.01
			corriemientoz=0.25
			base=0.06
			altura=0.06
		#Rango de X
		x1=splinecub(corrimientox,corrimientox+base,tf,n)
		x2=splinecub(corrimientox+base,corrimientox+base,tf,n)
		x3=splinecub(corrimientox+base,corrimientox,tf,n)
		x4=splinecub(corrimientox,corrimientox,tf,n)
		#Rango de z
		z1=splinecub(altura+corriemientoz,altura+corriemientoz,tf,n)
		z2=splinecub(altura+corriemientoz,corriemientoz,tf,n)
		z3=splinecub(corriemientoz,corriemientoz,tf,n)
		z4=splinecub(corriemientoz,corriemientoz+altura,tf,n)
		coordenadas=[[x1,z1],[x2,z2],[x3,z3],[x4,z4]]
		return coordenadas

	def trazo_circulo(self,n,tf,codo):
		#Parametros del circulo
		if codo =="True":
			radio =0.04
			yc=0.28 #Valor desplazar el centro del circulo en y
			xc=-0.025 #Valor desplazar el centro del circulo en x

		else:
			radio =0.04
			yc=0.285 #Valor desplazar el centro del circulo en y
			xc=0.025 #Valor desplazar el centro del circulo en x

		trayectoria_x= np.array(splinecub(-radio+xc,radio+xc,tf,n))
		trayectoria_x2= np.array(splinecub(radio+xc,-radio+xc,tf,n))
		z_pos=np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc
		z_neg=(-1*np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc)
		coordenadas=[[trayectoria_x,z_pos],[trayectoria_x2,z_neg]]
		return coordenadas

	def trazo_triangulo(self,n,tf,codo):
		#Parametros del triangulo
		if codo =="True":
			limiteiz=-0.01
			limiteinf=0.25
			base=-0.09
			altura=0.07
		
		else:
			limiteiz=0.01
			limiteinf=0.25
			base=0.09
			altura=0.07
			
		limitede=limiteiz+base

		x1=splinecub(limiteiz,(limitede+limiteiz)/2,tf,n)
		x2=splinecub((limiteiz+limitede)/2,limiteiz+base,tf,n)
		x3=splinecub(limiteiz+base,limiteiz,tf,n)

		z1=splinecub(limiteinf,altura+limiteinf,tf,n)
		z2=splinecub(altura+limiteinf,limiteinf,tf,n)
		z3=splinecub(limiteinf,limiteinf,tf,n)
		coordenadas=[[x1,z1],[x2,z2],[x3,z3]]
		return coordenadas

	def calculos(self,coordenadas,n,codo):
			#Dimensiones de Openbotv1
			L1=0.094
			L2=0.262
			result=[]
			dynamixel_angulos=[]
			for punto in coordenadas:
				for i in range (n):	
					try:
						x=punto[0][i]
						z=punto[1][i]
						if codo == "False":
							cinematica=calculos_cinematica_inversa.cal_2GDL_arri(punto[0][i],punto[1][i],L1,L2)
							
						if codo ==  "True":
							cinematica=calculos_cinematica_inversa.cal_2GDL_aba(punto[0][i],punto[1][i],L1,L2)
						
						if x > 0 and x>= 0.15:
							q1_fix=-cinematica[0]+0.17
							q2_fix=-cinematica[1]+0.05

						elif x < 0 and x <= -0.15:
							q1_fix=-cinematica[0]-0.17
							q2_fix=-cinematica[1]-0.05

						else:
							q1_fix=-cinematica[0]
							q2_fix=-cinematica[1]
						
						result.append(cinematica)
						dynamixel_angulos.append([-q1_fix,-q2_fix])

					except ValueError :
						print("Error Matematico, Punto por fuera del espacio de trabajo")
						print("Error en la iteraciòn ",i)
						result="Error"
			return result,dynamixel_angulos


	### Funciones de trayectorias:
	def trayectoria_real(self,figura,codo):    
		tf=10
		n=25
		if figura== "cuadrado":
			coordenadas=self.trazo_cuadrado(n,tf,codo)
		if figura== "triangulo":
			coordenadas=self.trazo_triangulo(n,tf,codo)
		if figura== "circulo":
			coordenadas=self.trazo_circulo(n,tf,codo)

		result,dynamixel_angulos=self.calculos(coordenadas,n,codo)
		# Creating the marker for RVIZ visualization
		markerArray,marker=trayectorie_2dof.Create_markers()
		#Estructura de Rviz
		id=0
		contador=0
		#movemos el robot al primer punto
		Dynamixel= Dynamixel_trayectoria()
		Dynamixel.main()
		Dynamixel.punto_inicial_trayectoria("si","si",'no',dynamixel_angulos)

		input()
		for angulos in result:
				q1=angulos[0]
				q2=angulos[1]
				self.move_openbot_toposition(q1,q2)
                #Angulos para dynamixel
				q1_d= dynamixel_angulos[contador][0]
				q2_d= dynamixel_angulos[contador][1]
                #Conversiones
				conver_q1=int( 512-q1_d*1023/(5/3*pi) ) #Q1
				conver_q2=int( 512-q2_d*1023/(5/3*pi) ) #Q2
				Dynamixel.mover_dynamixel([conver_q1,conver_q2,'no'])
				
				markerArray.markers.append(marker)
				for m in markerArray.markers:
					m.id = id
					id += 1
				self.pub_marker.publish(markerArray)
				contador+=1


	def trayectoria_simulada(self,figura,codo):    
		tf=10
		n= 25
		if figura== "cuadrado":
			coordenadas=self.trazo_cuadrado(n,tf,codo)
		if figura== "triangulo":
			coordenadas=self.trazo_triangulo(n,tf,codo)
		if figura== "circulo":
			coordenadas=self.trazo_circulo(n,tf,codo)

		result,dynamixel_angulos=self.calculos(coordenadas,n,codo)
		#rate=rospy.Rate(n/tf)
		# Creating the marker for RVIZ visualization
		markerArray,marker=trayectorie_2dof.Create_markers()
		#Estructura de Rviz
		id=0
		for angulos in result:
				q1=angulos[0]
				q2=angulos[1]
				self.move_openbot_toposition(q1,q2)
				#rate.sleep()
				markerArray.markers.append(marker)
				for m in markerArray.markers:
					m.id = id
					id += 1
				self.pub_marker.publish(markerArray)

	def trayectoria_solo_real(self,figura,codo):    
		tf=10
		if figura== "cuadrado":
			n= 70
			coordenadas=self.trazo_cuadrado(n,tf,codo)
		if figura== "triangulo":
			n= 70
			coordenadas=self.trazo_triangulo(n,tf,codo)
		if figura== "circulo":
			n= 150
			coordenadas=self.trazo_circulo(n,tf,codo)

		result,dynamixel_angulos=self.calculos(coordenadas,n,codo)
		contador=0
		#movemos el robot al primer punto

		Dynamixel= Dynamixel_trayectoria()
		Dynamixel.main()
		Dynamixel.punto_inicial_trayectoria("si","si",'no',dynamixel_angulos)
		input()
		for i in range(3):
			for angulos in result:
					#Angulos para dynamixel
					q1_d= dynamixel_angulos[contador][0]
					q2_d= dynamixel_angulos[contador][1]
					#Conversiones
					conver_q1=int( 512-q1_d*1023/(5/3*pi) ) #Q1
					conver_q2=int( 512-q2_d*1023/(5/3*pi) ) #Q2
					Dynamixel.mover_dynamixel([conver_q1,conver_q2,'no'])
					contador+=1
			contador=0

			
	



if __name__=="__main__":
	openbot_object=trayectorie_2dof()
	try:
		openbot_object.move_init_pos()     
		openbot_object.move_dynamic_position()   		
	except rospy.ROSInterruptException:
		pass

