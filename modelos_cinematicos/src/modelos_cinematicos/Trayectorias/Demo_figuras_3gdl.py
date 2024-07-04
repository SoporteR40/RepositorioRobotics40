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
from modelos_cinematicos.cfg import figuras_3gConfig as trayectoria_3g

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



class trayectorie_3dof(object):
	
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

  			
	def move_openbot_toposition(self,q1,q2,q3):
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
	

	def move_dynamic_position(self):
		""" Move the robot on a position (x,z) with dynamic reconfigure"""
		#command to launch dynamic reconfigure : rosrun rqt_gui rqt_gui -s reconfigure
			
		def callback_ready(trayectoria_3g, level):
			openbot_object=trayectorie_3dof()

			if "{Workspace}".format(**trayectoria_3g)=="True":
				qy=float("{Angle_qy}".format(**trayectoria_3g))
				self.move_init_pos()
				workspace=Espacio_de_trabajo()
				#Traigo las coordenadas
				coordenadas=workspace.workspace_3g_qy(qy)
				self.Rviz_workspace_completo(coordenadas)
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("1")
				print("1")
				rate.sleep()

			if "{Borrar_workspace}".format(**trayectoria_3g)=="True":
				self.Delete_marker()
				rate = rospy.Rate(10) # 10hz
				self.pub_configuracion.publish("2")
				print("2")
				rate.sleep()

			if "{Grafica_Python}".format(**trayectoria_3g)=="True":
				qy=float("{Angle_qy}".format(**trayectoria_3g))
				if "{Cuadrado}".format(**trayectoria_3g)=="True" and "{Triangulo}".format(**trayectoria_3g)=="False" and "{Circulo}".format(**trayectoria_3g)=="False":
					figura="1"
				elif "{Cuadrado}".format(**trayectoria_3g)=="False" and "{Triangulo}".format(**trayectoria_3g)=="True"and "{Circulo}".format(**trayectoria_3g)=="False":
					figura="2"
				elif "{Cuadrado}".format(**trayectoria_3g)=="False" and "{Triangulo}".format(**trayectoria_3g)=="False"and "{Circulo}".format(**trayectoria_3g)=="True":
					figura="3"
				else:
					figura="no"

				Graficas=Graficas_Espacio_de_trabajo_figuras()
				Graficas.grafica_workspace_3g_qy_figuras(qy,figura)
		


			if "{Ready}".format(**trayectoria_3g)=="True":  
					qy=float("{Angle_qy}".format(**trayectoria_3g))
					if "{Cuadrado}".format(**trayectoria_3g)=="True" and "{Triangulo}".format(**trayectoria_3g)=="False" and "{Circulo}".format(**trayectoria_3g)=="False":
						figura="cuadrado"
						if "{Simulacion}".format(**trayectoria_3g)=="True" and "{Dynamixel}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="False":
							openbot_object.trayectoria_simulada(figura,qy)
						if "{Dynamixel}".format(**trayectoria_3g)=="True" and "{Simulacion}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="False":
							openbot_object.trayectoria_real(figura,qy)
						if "{Dynamixel}".format(**trayectoria_3g)=="False" and "{Simulacion}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="True":
							openbot_object.trayectoria_solo_real(figura,qy)

					if "{Cuadrado}".format(**trayectoria_3g)=="False" and "{Triangulo}".format(**trayectoria_3g)=="True"and "{Circulo}".format(**trayectoria_3g)=="False":
						figura="triangulo"
						if "{Simulacion}".format(**trayectoria_3g)=="True" and "{Dynamixel}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="False":
							openbot_object.trayectoria_simulada(figura,qy)
						if "{Dynamixel}".format(**trayectoria_3g)=="True" and "{Simulacion}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="False":
							openbot_object.trayectoria_real(figura,qy)
						if "{Dynamixel}".format(**trayectoria_3g)=="False" and "{Simulacion}".format(**trayectoria_3g)=="False" and"{Solo_Dynamixel}".format(**trayectoria_3g)=="True":
							openbot_object.trayectoria_solo_real(figura,qy)

					if "{Cuadrado}".format(**trayectoria_3g)=="False" and "{Triangulo}".format(**trayectoria_3g)=="False"and "{Circulo}".format(**trayectoria_3g)=="True":
						figura="circulo"
						if "{Simulacion}".format(**trayectoria_3g)=="True" and "{Dynamixel}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="False":
							openbot_object.trayectoria_simulada(figura,qy)
						if "{Dynamixel}".format(**trayectoria_3g)=="True" and "{Simulacion}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="False":
							openbot_object.trayectoria_real(figura,qy)
						if "{Dynamixel}".format(**trayectoria_3g)=="False" and "{Simulacion}".format(**trayectoria_3g)=="False" and "{Solo_Dynamixel}".format(**trayectoria_3g)=="True":
							openbot_object.trayectoria_solo_real(figura,qy)
					
			else:
				pass
			return trayectoria_3g

		srv = Server(trayectoria_3g, callback_ready)
		rospy.spin()

	def trazo_cuadrado(self,n,tf):
		#Parametros del cuadrado
		corrimientox=0.06
		corriemientoz=0.145
		base=0.05
		altura=0.05

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

	def trazo_circulo(self,n,tf):
		#Parametros del circulo
		radio =0.028
		yc=0.17 #Valor desplazar el centro del circulo en y
		xc=0.08 #Valor desplazar el centro del circulo en x
		trayectoria_x= np.array(splinecub(-radio+xc,radio+xc,tf,n))
		trayectoria_x2= np.array(splinecub(radio+xc,-radio+xc,tf,n))
		z_pos=np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc
		z_neg=(-1*np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc)

		coordenadas=[[trayectoria_x,z_pos],[trayectoria_x2,z_neg]]
		return coordenadas
	
	def trazo_triangulo(self,n,tf):
		#Parametros del triangulo
		limiteiz=0.06
		limiteinf=0.145
		base=0.06
		altura=0.06

		limitede=limiteiz+base

		x1=splinecub(limiteiz,(limitede+limiteiz)/2,tf,n)
		x2=splinecub((limiteiz+limitede)/2,limiteiz+base,tf,n)
		x3=splinecub(limiteiz+base,limiteiz,tf,n)

		z1=splinecub(limiteinf,altura+limiteinf,tf,n)
		z2=splinecub(altura+limiteinf,limiteinf,tf,n)
		z3=splinecub(limiteinf,limiteinf,tf,n)
		coordenadas=[[x1,z1],[x2,z2],[x3,z3]]
		return coordenadas

	def calculos(self,coordenadas,qy,n):
			#Dimensiones de Openbotv1
			L1=0.094
			L2=0.112
			L3=0.15
			result=[]
			dynamixel_angulos=[]
			contador=0
			for punto in coordenadas:
				for i in range (n):	
					try:
						print("Contador: ",contador)
						x=punto[0][i]
						z=punto[1][i]
						if qy<=90:
							codo="False"
						if qy>90:
							codo="True"

						if codo == "False":
							cinematica=calculos_cinematica_inversa.cal_3GDL_arri(punto[0][i],punto[1][i],qy,L1,L2,L3)
							
						if codo ==  "True":
							cinematica=calculos_cinematica_inversa.cal_3GDL_aba(punto[0][i],punto[1][i],qy,L1,L2,L3)
						
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
						dynamixel_angulos.append([-q1_fix,-q2_fix,cinematica[2]])

						contador+=1
					except ValueError :
						print("Error Matematico, Punto por fuera del espacio de trabajo")
						print("Error en la iteraciòn ",i)
						result="Error"
			
			return result,dynamixel_angulos


	### Funciones de trayectorias:
	def trayectoria_real(self,figura,qy):
		tf=10
		n=25
		if figura== "cuadrado":
			coordenadas=self.trazo_cuadrado(n,tf)
		if figura== "triangulo":
			coordenadas=self.trazo_triangulo(n,tf)
		if figura== "circulo":
			coordenadas=self.trazo_circulo(n,tf)

		result,dynamixel_angulos=self.calculos(coordenadas,qy,n)
		# Creating the marker for RVIZ visualization
		markerArray,marker=trayectorie_3dof.Create_markers()
		#Estructura de Rviz
		id=0
		contador=0
		#movemos el robot al primer punto
		Dynamixel= Dynamixel_trayectoria()
		Dynamixel.main()
		Dynamixel.movimiento_inicial('no','no','no','no','no',-0.345)
		Dynamixel.punto_inicial_trayectoria("si","si","si",dynamixel_angulos)

		input()

		for angulos in result:
				q1=angulos[0]
				q2=angulos[1]
				q3=angulos[2]
				self.move_openbot_toposition(q1,q2,q3)
                #Angulos para dynamixel
				q1_d= dynamixel_angulos[contador][0]
				q2_d= dynamixel_angulos[contador][1]
				q3_d=dynamixel_angulos[contador][2]
                #Conversiones
				conver_q1=int( 512-q1_d*1023/(5/3*pi) ) #Q1
				conver_q2=int( 512-q2_d*1023/(5/3*pi) ) #Q2
				conver_q3=int( 512+q3_d*1023/(5/3*pi) ) #Q3
				print("Movimiento: ",contador)
				print([q1_d,q2_d,q3_d])
				Dynamixel.mover_dynamixel([conver_q1,conver_q2,conver_q3])
				print(" ")

				markerArray.markers.append(marker)
				for m in markerArray.markers:
					m.id = id
					id += 1
				self.pub_marker.publish(markerArray)
				contador+=1


	def trayectoria_simulada(self,figura,qy):    
		tf=10
		n=25
		if figura== "cuadrado":
			coordenadas=self.trazo_cuadrado(n,tf)
		if figura== "triangulo":
			coordenadas=self.trazo_triangulo(n,tf)
		if figura== "circulo":
			coordenadas=self.trazo_circulo(n,tf)

		result,dynamixel_angulos=self.calculos(coordenadas,qy,n)
		rate=rospy.Rate(n/tf)
		# Creating the marker for RVIZ visualization
		markerArray,marker=trayectorie_3dof.Create_markers()
		#Estructura de Rviz
		id=0
		for angulos in result:
				q1=angulos[0]
				q2=angulos[1]
				q3=angulos[2]
				self.move_openbot_toposition(q1,q2,q3)
				markerArray.markers.append(marker)
				for m in markerArray.markers:
					m.id = id
					id += 1
				self.pub_marker.publish(markerArray)

	def trayectoria_solo_real(self,figura,qy):    
		tf=10
		if figura== "cuadrado":
			n= 70
			coordenadas=self.trazo_cuadrado(n,tf)
		if figura== "triangulo":
			n= 70
			coordenadas=self.trazo_triangulo(n,tf)
		if figura== "circulo":
			n= 200
			coordenadas=self.trazo_circulo(n,tf)

		result,dynamixel_angulos=self.calculos(coordenadas,qy,n)
		contador=0
		#movemos el robot al primer punto

		Dynamixel= Dynamixel_trayectoria()
		Dynamixel.main()
		Dynamixel.movimiento_inicial('no','no','no','no','no',-0.345)
		Dynamixel.punto_inicial_trayectoria("si","si","si",dynamixel_angulos)
		input()
		for i in range(3):
			for angulos in result:
                #Angulos para dynamixel
				q1_d= dynamixel_angulos[contador][0]
				q2_d= dynamixel_angulos[contador][1]
				q3_d=dynamixel_angulos[contador][2]
                #Conversiones
				conver_q1=int( 512-q1_d*1023/(5/3*pi) ) #Q1
				conver_q2=int( 512-q2_d*1023/(5/3*pi) ) #Q2
				conver_q3=int( 512+q3_d*1023/(5/3*pi) ) #Q3
				Dynamixel.mover_dynamixel([conver_q1,conver_q2,conver_q3])
				contador+=1

			contador=0

	



if __name__=="__main__":
	openbot_object=trayectorie_3dof()
	try:
		openbot_object.move_init_pos()     
		openbot_object.move_dynamic_position()   		
	except rospy.ROSInterruptException:
		pass

