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

N=250
Tf=10
x1=0.2
y1=0.1
z1=0.25
x2=0.2
y2=0.12
z2=0.25
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
        conver_q1=( 512-q1*1023/(5/3*pi) ) #Q1
        conver_q2=( 512-q2*1023/(5/3*pi) ) #Q2
        conver_q3=( 512-q3*1023/(5/3*pi) ) #Q3
        conver_q4=( 512+q4 *1023/(5/3*pi))  #Q4
        conversiones.append([conver_q1,conver_q2,conver_q3,conver_q4])

print(result)


print(conversiones)