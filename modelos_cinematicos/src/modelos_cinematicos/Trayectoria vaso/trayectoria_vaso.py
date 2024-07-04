#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
# Computing 
from math import acos,atan,sqrt,pi 
import numpy as np 

#Clases y funciones
from inverse_kinematic_computing import Inverse_kinematic_compute
from splinecub_puntos import splinecub
from splinecub_dynamixel import splinecub_2
from dynamixel_sdk import * 
from Espacio_de_trabajo.Espacio_de_trabajo_3GDL_Codo_arriba import graficar_espacio_de_trabajo
from package_dynamixel.DynamixelSDK.ros.dynamixel_sdk_examples.src.mover_openbot_dynamixel import mover_Dynamixel

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
DXL1_ID                     = 13 ##q1                
DXL2_ID                     = 14 ##q2          
DXL3_ID                     = 15 ##q3          
DXL4_ID                     = 17 #q4
DXL6_ID                     = 16 #giro de articulaciòn superior
DXL5_ID                     = 18 ##pinza 
 

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

	def trayectoria_real(self,N,Tf,x1,y1,z1,x2,y2,z2,qyg):    
			#Dimensiones de Openbotv1
			L1=0.33
			L2 = 0.094
			L3 = 0.115
			L4 = 0.155
			qy=qyg*pi/180
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
					conver_q4=int( 512+(q4) *1023/(5/3*pi))  #Q4
					conversiones.append([conver_q1,conver_q2,conver_q3,conver_q4])
			Dynamixel=mover_Dynamixel()
			#self.movimiento_inicial(conversiones)
			Dynamixel.mover_motores(conversiones)
			#self.mover_dynamixel(conversiones)

		

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
		n0=250
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
			rate=rospy.Rate(650/5)
			rate.sleep()


	def Recoger_vaso(self):
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
	openbot_object=trazar_trayectoria()
	openbot_object.clearConsole()
	#openbot_object.main()
	#desicion=openbot_object.menu()
	#openbot_object.abrir_pinza()
	input()
	#openbot_object.cerrar_pinza()
	openbot_object.Hacer_trayectoria(250,10,0.2,0.2,0.1,0.12,0.25,0.25)
	#if desicion == 1:
		#openbot_object.main()
		#openbot_object.movimiento_inicial(conversiones)
		#rate=rospy.Rate(650/5)
		#rate.sleep()
		#for puntos in conversiones:
				#openbot_object.trayectoria_t0(puntos)
				#rate=rospy.Rate(650/5)
				#rate.sleep()
		

