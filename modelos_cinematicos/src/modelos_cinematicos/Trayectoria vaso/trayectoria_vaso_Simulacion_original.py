#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import time

# Computing 
from math import acos,atan,sqrt,pi 
import numpy as np

# Message 
from std_msgs.msg import Float64
from openbot_inverse_kinematic.msg import angle

# Server
from dynamic_reconfigure.server import Server

# Configuration 
from openbot_inverse_kinematic.cfg import Angle_jointConfig as config_angle
from openbot_inverse_kinematic.cfg import position3dofConfig as config_position

# TF
import tf2_ros
import tf2_msgs.msg

#Clases y funciones
from inverse_kinematic_computing import Inverse_kinematic_compute
from splinecub_puntos import splinecub
from splinecub_dynamixel import splinecub_2
from dynamixel_sdk import * 
from Espacio_de_trabajo.Espacio_de_trabajo_3GDL_Codo_arriba import graficar_espacio_de_trabajo

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Tabla de control de Dynamixel
ADDR_MX_TORQUE_ENABLE      = 24               
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4
# Protocol version
PROTOCOL_VERSION            = 1.0               
# Default setting
DXL1_ID                     = 13  ##q1                
DXL2_ID                     = 14   ##q2          
DXL3_ID                     = 15   ##q3          
DXL4_ID                     = 17   ##q4
DXL5_ID                     = 18   ##pinza 
 

BAUDRATE                    = 1000000           
DEVICENAME                  = '/dev/ttyUSB0'    
TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                 

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)
# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

class trazar_trayectoria (object):

	def __init__(self):
		"""Initialisation of the object"""
		# Initilisation of the node 
		rospy.init_node('node_move_vaso')
		rospy.loginfo('Initialising openbot...')
		self.pub_arm1 = rospy.Publisher('/openbot_v1/arm1_arm2_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm2 = rospy.Publisher('/openbot_v1/arm2_arm3_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm3 = rospy.Publisher('/openbot_v1/arm3_arm4_joint_position_controller/command', Float64, queue_size=1)
		self.pub_arm4 = rospy.Publisher('/openbot_v1/arm4_clamp1_joint_position_controller/command', Float64, queue_size=1)
		self.pub_clamp = rospy.Publisher('/openbot_v1/clamp1_clamp2_joint_position_controller/command', Float64, queue_size=1)
		self.pub_base = rospy.Publisher('/openbot_v1/base_arm1_joint_position_controller/command', Float64, queue_size=1)
		self.pub_angle = rospy.Publisher('/angle_joint',angle,queue_size=10)
		rospy.on_shutdown(self.shutdownhook)

	def clearConsole(self):
		command = 'clear'
		if os.name in ('nt', 'dos'):  
			command = 'cls'
		os.system(command)
	
	def shutdownhook(self):
		self.ctrl_c = True

	def menu(self):
			print( """
Menu
Bienvenido, Digite la accciòn a realizar:
(1)Recoger Vaso
(2)Acercar a boca
(3)Dejar Vaso en posiciòn inicial
(4)Salir""")

			desicion=int(input())
			if desicion==1:
				self.Recoger_vaso()
			elif desicion==2:
				self.Acercar_a_boca()
			elif desicion==3:
				self.dejar_vaso()
			else:
				print("Opcion Equivocada, Presione Enter para continuar")
				input()
				self.menu()

			return desicion






	def main(self):
		if portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()
		# Set port baudrate
		if portHandler.setBaudRate(BAUDRATE):
			print("Succeeded to change the baudrate")
		else:
			print("Failed to change the baudrate")
			print("Press any key to terminate...")
			getch()
			quit()
		#Verfificamos si todos los dynamixel estan conectados
		#######################################################################################################

		# Enable Dynamixel#1 Torque
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			print("Dynamixel#%d has been successfully connected" % DXL1_ID)

		# Enable Dynamixel#2 Torque
		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			print("Dynamixel#%d has been successfully connected" % DXL2_ID)
		# Enable Dynamixel#3 Torque

		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			print("Dynamixel#%d has been successfully connected" % DXL3_ID)
		# Enable Dynamixel#4 Torque

		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			print("Dynamixel#%d has been successfully connected" % DXL4_ID)

		dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
		else:
			print("Dynamixel#%d has been successfully connected" % DXL5_ID)












	def trayectoria_real(self,N,Tf,x1,y1,z1,x2,y2,z2):    
		#Dimensiones de Openbotv1
		L1=0.33
		L2 = 0.094
		L3 = 0.115
		L4 = 0.155
		qy=45*pi/180
		#Se generan los puntos entre X1 y X2 / Z1 y Z2
		trayectoria_x = splinecub(x1,x2,Tf,N)
		trayectoria_y = splinecub(y1,y2,Tf,N)
		trayectoria_z = splinecub(z1,z2,Tf,N)
		#Calculamos cinematica inversa para cada punto
		#Definimos los vectores
		result=[]
		conversiones= []
		for i in range (N):
				
				cinematica=Inverse_kinematic_compute.Calcul_angle4GDL_arr(trayectoria_x[i],trayectoria_y[i],trayectoria_z[i],qy,L1,L2,L3,L4)
				result.append(cinematica)
			
		for angulos in result:
				q1=angulos[0]
				q2=angulos[1]
				q3=angulos[2]
				q4=angulos[3]
				conver_q1=int( 512-q1*1023/(5/3*pi) ) #Q1
				conver_q2=int( 512-q2*1023/(5/3*pi) ) #Q2
				conver_q3=int( 512-q3*1023/(5/3*pi) ) #Q3
				conver_q4=int( 512+q4 *1023/(5/3*pi))  #Q4
				conversiones.append([conver_q1,conver_q2,conver_q3,conver_q4])
		
		self.movimiento_inicial(conversiones)
		#rate=rospy.Rate(250/10)
		#rate.sleep()
		self.mover_dynamixel(conversiones)

	def trayectoria_simulada(self,N,Tf,x1,y1,z1,x2,y2,z2):    
		#Dimensiones de Openbotv1
		L1=0.33
		L2 = 0.094
		L3 = 0.115
		L4 = 0.155
		qy=45*pi/180
		#Se generan los puntos entre X1 y X2 / Z1 y Z2
		trayectoria_x = splinecub(x1,x2,Tf,N)
		trayectoria_y = splinecub(y1,y2,Tf,N)
		trayectoria_z = splinecub(z1,z2,Tf,N)
		#Calculamos cinematica inversa para cada punto
		#Definimos los vectores
		result=[]
		conversiones= []
		for i in range (N):
				
				cinematica=Inverse_kinematic_compute.Calcul_angle4GDL_arr(trayectoria_x[i],trayectoria_y[i],trayectoria_z[i],qy,L1,L2,L3,L4)
				result.append(cinematica)
			
		for angulos in result:
				q1=angulos[0]
				q2=angulos[1]
				q3=angulos[2]
				q4=angulos[3]
				self.move_openbot_toposition(q1,q2,q3,q4)
			
		
		












						
		
	###Funciones Dynamixel


	def abrir_pinza(self):
		n0=300  #Podrian dejarse fijas (?)
		dxl5_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL5_ID, ADDR_MX_PRESENT_POSITION)
		th50 = splinecub_2(dxl5_present_position0[0],100,10,n0)

		for k in range (0,n0):
			# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
			param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th50[k])), DXL_HIBYTE(DXL_LOWORD(th50[k])), DXL_LOBYTE(DXL_HIWORD(th50[k])), DXL_HIBYTE(DXL_HIWORD(th50[k]))]
			dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID,param_goal_position)
			if dxl_addparam_result != True:
				print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
				quit()
			# Syncwrite goal position
			dxl_comm_result = groupSyncWrite.txPacket()
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            #Clear syncwrite parameter storage
			groupSyncWrite.clearParam()

	def cerrar_pinza(self):
		n0=300
		dxl5_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION)
		th50 = splinecub_2(dxl5_present_position0[0],490,10,n0)
		for k in range (0,n0):
			# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
			param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th50[k])), DXL_HIBYTE(DXL_LOWORD(th50[k])), DXL_LOBYTE(DXL_HIWORD(th50[k])), DXL_HIBYTE(DXL_HIWORD(th50[k]))]
			dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID,param_goal_position)
			if dxl_addparam_result != True:
				print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
				quit()
			# Syncwrite goal position
			dxl_comm_result = groupSyncWrite.txPacket()
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            #Clear syncwrite parameter storage
			groupSyncWrite.clearParam()

	def movimiento_inicial(self,conversiones):
		n0=100
		dxl1_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
		th10 = splinecub_2(dxl1_present_position0[0],conversiones[0][0],10,n0)
		dxl2_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
		th20 = splinecub_2(dxl2_present_position0[0],conversiones[0][1],10,n0)
		dxl3_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
		th30 = splinecub_2(dxl3_present_position0[0],conversiones[0][2],10,n0)
		dxl4_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION)
		th40 = splinecub_2(dxl4_present_position0[0],conversiones[0][3],10,n0)

		for k in range (0,n0):
			# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
			param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th10[k])), DXL_HIBYTE(DXL_LOWORD(th10[k])), DXL_LOBYTE(DXL_HIWORD(th10[k])), DXL_HIBYTE(DXL_HIWORD(th10[k]))]
			dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID,param_goal_position)
			if dxl_addparam_result != True:
				print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
				quit() 
			# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
			param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(th20[k])), DXL_HIBYTE(DXL_LOWORD(th20[k])), DXL_LOBYTE(DXL_HIWORD(th20[k])), DXL_HIBYTE(DXL_HIWORD(th20[k]))]
			dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID,param_goal_position_2)
			if dxl_addparam_result != True:
				print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
				quit()
			# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
			param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(th30[k])), DXL_HIBYTE(DXL_LOWORD(th30[k])), DXL_LOBYTE(DXL_HIWORD(th30[k])), DXL_HIBYTE(DXL_HIWORD(th30[k]))]
			dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID,param_goal_position_3)
			if dxl_addparam_result != True:
				print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
				quit()

			# Add Dynamixel#4 goal position value to the Syncwrite parameter storage
			param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(th40[k])), DXL_HIBYTE(DXL_LOWORD(th40[k])), DXL_LOBYTE(DXL_HIWORD(th40[k])), DXL_HIBYTE(DXL_HIWORD(th40[k]))]
			dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID,param_goal_position_4)
			if dxl_addparam_result != True:
				print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
				quit()
            # Syncwrite goal position
			dxl_comm_result = groupSyncWrite.txPacket()
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            #Clear syncwrite parameter storage
			groupSyncWrite.clearParam()
			#Delay entre cada movimiento
			rate=rospy.Rate(650/10)
			rate.sleep()



	def mover_dynamixel(self,puntos):
		for posicion in puntos:
			if posicion[0] != 'none':
				# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
				param_goal_position = [DXL_LOBYTE(DXL_LOWORD(posicion[0])), DXL_HIBYTE(DXL_LOWORD(posicion[0])), DXL_LOBYTE(DXL_HIWORD(posicion[0])), DXL_HIBYTE(DXL_HIWORD(posicion[0]))]
				dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID,param_goal_position)
				if dxl_addparam_result != True:
					print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
					quit()
			if posicion[1] != 'none':
				# Add Dynamixel#2 goal position value to the Syncwrite parameter storage
				param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(posicion[1])), DXL_HIBYTE(DXL_LOWORD(posicion[1])), DXL_LOBYTE(DXL_HIWORD(posicion[1])), DXL_HIBYTE(DXL_HIWORD(posicion[1]))]
				dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID,param_goal_position_2)
				if dxl_addparam_result != True:
					print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
					quit()
			if posicion[2] != 'none':
				# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
				param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(posicion[2])), DXL_HIBYTE(DXL_LOWORD(posicion[2])), DXL_LOBYTE(DXL_HIWORD(posicion[2])), DXL_HIBYTE(DXL_HIWORD(posicion[2]))]
				dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID,param_goal_position_3)
				if dxl_addparam_result != True:
					print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
					quit()

			if posicion[3] != 'none':
				# Add Dynamixel#3 goal position value to the Syncwrite parameter storage
				param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(posicion[3])), DXL_HIBYTE(DXL_LOWORD(posicion[3])), DXL_LOBYTE(DXL_HIWORD(posicion[3])), DXL_HIBYTE(DXL_HIWORD(posicion[3]))]
				dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID,param_goal_position_4)
				if dxl_addparam_result != True:
					print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
					quit()					
			# Syncwrite goal position
			dxl_comm_result = groupSyncWrite.txPacket()
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
			#Clear syncwrite parameter storage
			groupSyncWrite.clearParam()

			#Delay entre cada movimiento
			rate=rospy.Rate(650/10)
			rate.sleep()











	#### Funciones de simulacion

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
		""" Move the robot to the position (x,z,qy) with the angle we compute"""
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
		rospy.loginfo('Moving openbot to the position (x,z,qy) ...')
		self.move_openbot(angle_joints)
		rospy.sleep(1)
		tfBuffer = tf2_ros.Buffer() # To know the position of the effector
		listener = tf2_ros.TransformListener(tfBuffer)
		trans = tfBuffer.lookup_transform('world', 'effector_link_tf', rospy.Time(),rospy.Duration(10))			
		rospy.loginfo("The position in x is : %.2f",-(trans.transform.translation.x-0.005)) # Print the position of the effector
		rospy.loginfo("The position in y is : %.2f",-(trans.transform.translation.y+0.003))
		rospy.loginfo("The position in z is : %.2f",trans.transform.translation.z-0.16)










	def Recoger_vaso(self):
		
		desicion=int(input("¿Desea ver la simulaciòn de la trayectoria? (1 Si (2 No  "))
		if desicion == 1:
			self.Recoger_vaso_simulado()

		elif desicion == 2:
			#self.trayectoria_real(100,10,x1=0.2,y1=0.1,z1=0.25,x2=0.2,y2=0.15,z2=0.25)
			self.abrir_pinza()
			self.trayectoria_real(100,10,x1=0.2,y1=-0.1,z1=0.25,x2=0.25,y2=-0.018,z2=0.14)
			input()
			self.trayectoria_real(100,10,x1=0.25,y1=-0.018,z1=0.14,x2=0.27,y2=-0.018,z2=0.14)		
			input("Trayectoria terminada, presione entenr para regresar a menu")
			self.cerrar_pinza()
			self.menu()
		else:
			print("Opcion Equivocada, Presione Enter para continuar")
			input()
			self.Recoger_vaso()
		
		

	def Recoger_vaso_simulado(self):
		
		self.move_init_pos()
		#self.trayectoria_simulada(5,10,x1=0.2,y1=0.1,z1=0.25,x2=0.2,y2=0.15,z2=0.25)
		self.trayectoria_simulada(5,10,x1=0.2,y1=0.21,z1=0.25,x2=0.27,y2=0.035,z2=0.14)	

		input("Simulaciòn terminada, presione entenr para regresar a menu")
		self.Recoger_vaso()

if __name__=="__main__":
	openbot_object=trazar_trayectoria()
	openbot_object.clearConsole()
	openbot_object.main()
	openbot_object.menu()
	#openbot_object.abrir_pinza()
	#input()
	#openbot_object.cerrar_pinza()
	#openbot_object.Recoger_vaso
		#openbot_object.main()
		#openbot_object.movimiento_inicial(conversiones)
		#rate=rospy.Rate(650/5)
		#rate.sleep()
		#for puntos in conversiones:
				#openbot_object.trayectoria_t0(puntos)
				#rate=rospy.Rate(650/5)
				#rate.sleep()
		

