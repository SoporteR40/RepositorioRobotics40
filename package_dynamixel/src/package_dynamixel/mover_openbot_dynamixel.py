#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
import os
import rospy
import numpy as np
from math import acos,atan,sqrt,pi 
from package_dynamixel.splinecub_dynamixel import splinecub_2 
from dynamixel_sdk import *                    

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

#Cantidad de puntos 
n0=500
# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 13 ##q1                
DXL2_ID                     = 14 ##q2          
DXL3_ID                     = 15 ##q3          
DXL4_ID                     = 16 #q4
DXL5_ID                     = 17 ##q5 
DXL6_ID      				= 18   ##q6 (Pinza)        
              
BAUDRATE                    = 1000000           
DEVICENAME                  = '/dev/ttyUSB0'    

TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                 

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

class mover_Dynamixel(object):

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
        # Enable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL5_ID)

        # Enable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL6_ID)




    def movimiento_inicial(self,q1,q2,q3,q4,q5,q6):
        print(q1,q2,q3,q4,q5,q6)
        p1='no'
        p2='no'
        p3='no'
        p4='no'
        p5='no'
        p6='no'
        n0=100
        if q1!='no':
            p1=int( 512-q1*1023/(5/3*pi) )#q1
            dxl1_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
            th10 = splinecub_2(dxl1_present_position0[0],p1,10,n0)
        if q2!='no':
            p2=int( 512-q2*1023/(5/3*pi) ) #Q2
            dxl2_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
            th20 = splinecub_2(dxl2_present_position0[0],p2,10,n0)
        if q3!='no':
            p3=int( 512-q3*1023/(5/3*pi) ) #Q3
            dxl3_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
            th30 = splinecub_2(dxl3_present_position0[0],p3,10,n0)
        if q4!='no':
            p4=int( 512-q4 *1023/(5/3*pi))  #Q4
            dxl4_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION)
            th40 = splinecub_2(dxl4_present_position0[0],p4,10,n0)
        if q5!='no':
            p5=int( 512+q5*1023/(5/3*pi) )
            dxl5_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL5_ID, ADDR_MX_PRESENT_POSITION)
            th50 = splinecub_2(dxl5_present_position0[0],p5,10,n0)
        if q6!='no':
            p6=int( 512-q6*1023/(5/3*pi) )
            dxl6_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL6_ID, ADDR_MX_PRESENT_POSITION)
            th60 = splinecub_2(dxl6_present_position0[0],p6,10,n0)
        print('Posiciones destino de los motores')
        print(p1,p2,p3,p4,p5,p6)

        for k in range (0,n0):
            if q1!='no':
                # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th10[k])), DXL_HIBYTE(DXL_LOWORD(th10[k])), DXL_LOBYTE(DXL_HIWORD(th10[k])), DXL_HIBYTE(DXL_HIWORD(th10[k]))]
                dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID,param_goal_position)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
                    quit() 
            if q2!='no':
                # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
                param_goal_position_2 = [DXL_LOBYTE(DXL_LOWORD(th20[k])), DXL_HIBYTE(DXL_LOWORD(th20[k])), DXL_LOBYTE(DXL_HIWORD(th20[k])), DXL_HIBYTE(DXL_HIWORD(th20[k]))]
                dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID,param_goal_position_2)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
                    quit()

            if q3!='no':
                # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
                param_goal_position_3 = [DXL_LOBYTE(DXL_LOWORD(th30[k])), DXL_HIBYTE(DXL_LOWORD(th30[k])), DXL_LOBYTE(DXL_HIWORD(th30[k])), DXL_HIBYTE(DXL_HIWORD(th30[k]))]
                dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID,param_goal_position_3)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
                    quit()
            if q4!='no':
                # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
                param_goal_position_4 = [DXL_LOBYTE(DXL_LOWORD(th40[k])), DXL_HIBYTE(DXL_LOWORD(th40[k])), DXL_LOBYTE(DXL_HIWORD(th40[k])), DXL_HIBYTE(DXL_HIWORD(th40[k]))]
                dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID,param_goal_position_4)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
                    quit()

            if q5!='no':
                #Add Dynamixel#5 goal position value to the Syncwrite parameter storage
                param_goal_position_5 = [DXL_LOBYTE(DXL_LOWORD(th50[k])), DXL_HIBYTE(DXL_LOWORD(th50[k])), DXL_LOBYTE(DXL_HIWORD(th50[k])), DXL_HIBYTE(DXL_HIWORD(th50[k]))]
                dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID,param_goal_position_5)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
                    quit()

            if q6!='no':
                #Add Dynamixel#6 goal position value to the Syncwrite parameter storage
                param_goal_position_6 = [DXL_LOBYTE(DXL_LOWORD(th60[k])), DXL_HIBYTE(DXL_LOWORD(th60[k])), DXL_LOBYTE(DXL_HIWORD(th60[k])), DXL_HIBYTE(DXL_HIWORD(th60[k]))]
                dxl_addparam_result = groupSyncWrite.addParam(DXL6_ID,param_goal_position_6)
                if dxl_addparam_result != True:
                    print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
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


    def imprimir_angulo(self,q1,q2,q3,q4,q5,q6):
        print(q1,q2,q3,q4,q5,q6)
      








