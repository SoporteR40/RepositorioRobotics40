#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
import os
import numpy as np
import math
from splinecub_lab import splinecub

#import matplotlib.pyplot as plt

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

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
DXL1_ID                     = 13                 # Dynamixel#1 ID : 1
DXL2_ID                     = 14                 # Dynamixel#1 ID : 2
DXL3_ID                     = 15                 # Dynamixel#1 ID : 1
DXL4_ID                     = 16                 # Dynamixel#1 ID : 2
DXL5_ID                     = 17                 # Dynamixel#1 ID : 1
DXL6_ID                     = 18                 # Dynamixel#1 ID : 2
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM4'    # Check which port is being used on your controller

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE1  = 300        
DXL_MINIMUM_POSITION_VALUE2  = 500         
DXL_MINIMUM_POSITION_VALUE3  = 500        
DXL_MINIMUM_POSITION_VALUE4  = 300        
DXL_MINIMUM_POSITION_VALUE5  = 500        
DXL_MINIMUM_POSITION_VALUE6  = 500
DXL_MAXIMUM_POSITION_VALUE1  = 1000        
DXL_MAXIMUM_POSITION_VALUE2  = 1000         
DXL_MAXIMUM_POSITION_VALUE3  = 1000       
DXL_MAXIMUM_POSITION_VALUE4  = 1000       
DXL_MAXIMUM_POSITION_VALUE5  = 1000       
DXL_MAXIMUM_POSITION_VALUE6  = 1000         


DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

index = 0
#dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

#Sección donde defino las funciones
  
n=100
dthf1=300
dthf2=100
dthf3=250
dthf4=250
dthf5=100
dthf6=100
t=np.arange(0,n+1)
pi= math.pi
th1p=[]
th2p=[]
th3p=[]
th4p=[]
th5p=[]
th6p=[]

for i in t:
    th1p.append(dthf1*math.sin(2*pi/n*i)+512)
    th2p.append(dthf2*math.sin(2*pi/n*i)+512)
    th3p.append(-dthf3*math.sin(2*pi/n*i)+512)
    th4p.append(dthf4*math.sin(2*pi/n*i)+512)
    th5p.append(dthf5*math.sin(2*pi/n*i)+512)
    th6p.append(dthf6*math.sin(2*pi/n*i)+512)


th1p=np.array(th1p,int)
th2p=np.array(th2p,int)
th3p=np.array(th3p,int)
th4p=np.array(th4p,int)
th5p=np.array(th5p,int)
th6p=np.array(th6p,int)
#######################################################################################################


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

# Open port
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



# Enable Dynamixel#6 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL6_ID)









###################################################################################################



#Llamamos a trayectoria t0
print('Inicio de t0')

input("Please press the Enter key to proceed")

#Lectura de la posicion actual 
dxl1_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
dxl2_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
dxl3_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
dxl4_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION)
dxl5_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL5_ID, ADDR_MX_PRESENT_POSITION)
dxl6_present_position0 = packetHandler.read2ByteTxRx(portHandler, DXL6_ID, ADDR_MX_PRESENT_POSITION)

#Cantidad de puntos de n0
n0=30


th10 = splinecub(dxl1_present_position0[0],th1p[0],5,n0)
th20 = splinecub(dxl2_present_position0[0],th2p[0],5,n0)
th30 = splinecub(dxl3_present_position0[0],th3p[0],5,n0)
th40 = splinecub(dxl4_present_position0[0],th4p[0],5,n0)
th50 = splinecub(dxl5_present_position0[0],th5p[0],5,n0)
th60 = splinecub(dxl6_present_position0[0],th6p[0],5,n0)
#Imprimos las 3 primeros trayectorias t0




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
    # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
    param_goal_position_5 = [DXL_LOBYTE(DXL_LOWORD(th50[k])), DXL_HIBYTE(DXL_LOWORD(th50[k])), DXL_LOBYTE(DXL_HIWORD(th50[k])), DXL_HIBYTE(DXL_HIWORD(th50[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID,param_goal_position_5)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
        quit()

    # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
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

    #Leemos la posición final de los dynamixel
    dxl1_present_position = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
    dxl2_present_position = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
    dxl3_present_position = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)

    #print( dxl1_present_position)
    #print( dxl2_present_position)
    #print( dxl3_present_position)
    

print ('Fin de to')



input("Please press the Enter key to proceed")
###############################################################################################################
##Intento de gráfica




#########################################################################################
#Enviamos la trayectoria de seno

for k in t:

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th1p[k])), DXL_HIBYTE(DXL_LOWORD(th1p[k])), DXL_LOBYTE(DXL_HIWORD(th1p[k])), DXL_HIBYTE(DXL_HIWORD(th1p[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID,param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th2p[k])), DXL_HIBYTE(DXL_LOWORD(th2p[k])), DXL_LOBYTE(DXL_HIWORD(th2p[k])), DXL_HIBYTE(DXL_HIWORD(th2p[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID,param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()
    # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th3p[k])), DXL_HIBYTE(DXL_LOWORD(th3p[k])), DXL_LOBYTE(DXL_HIWORD(th3p[k])), DXL_HIBYTE(DXL_HIWORD(th3p[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID,param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
        quit()
    # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th4p[k])), DXL_HIBYTE(DXL_LOWORD(th4p[k])), DXL_LOBYTE(DXL_HIWORD(th4p[k])), DXL_HIBYTE(DXL_HIWORD(th4p[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID,param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
        quit()
    # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th5p[k])), DXL_HIBYTE(DXL_LOWORD(th5p[k])), DXL_LOBYTE(DXL_HIWORD(th5p[k])), DXL_HIBYTE(DXL_HIWORD(th5p[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID,param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
        quit()
    # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(th6p[k])), DXL_HIBYTE(DXL_LOWORD(th6p[k])), DXL_LOBYTE(DXL_HIWORD(th6p[k])), DXL_HIBYTE(DXL_HIWORD(th1p[k]))]
    dxl_addparam_result = groupSyncWrite.addParam(DXL6_ID,param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
        quit()
    
    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))


    #Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    #Leemos la posición final de los dynamixel
    dxl1_present_position = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
    dxl2_present_position = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
    dxl3_present_position = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)





##############################################################################################################
print ('termino la trayectoria del seno')
input("Please press the Enter key to proceed")


# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


# Disable Dynamixel#4 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#5 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


# Disable Dynamixel#6 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


# Close port
portHandler.closePort()

