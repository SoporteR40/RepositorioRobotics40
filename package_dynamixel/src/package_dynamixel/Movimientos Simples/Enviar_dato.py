#!/usr/bin/env python3                         
# encoding: utf-8
import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *



def nodo():                                                 #Definimos una función nodo
    mensaje = [100,200,300,400,500,600]
    i=0
    id=18                                  
    rospy.init_node('Enviar_datos_node')                       #Inicializamos nuestro nodo y le asignamos un nombre = nodo_publisher
    
    pub = rospy.Publisher('set_position', SetPosition, queue_size=10) 
    rate = rospy.Rate(2)    

    while not rospy.is_shutdown():                          #Bucle While - hasta pulsar Ctrl-C
        print('Se esta enviado el dato',mensaje[i], 'en la id :', id)
        pub.publish( id, mensaje[i] ) 

        
        if i==(len(mensaje)-1):
            break
        i=i+1
        rate.sleep()                              
             

        
                               

if __name__ == '__main__':                                  #Llamamos a la función principal main
    try:
        nodo()                                              # Lamamos a la función nodo
    except rospy.ROSInterruptException:                     # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
        pass