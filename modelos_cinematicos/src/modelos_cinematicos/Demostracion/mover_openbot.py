#!/usr/bin/env python3                         
# encoding: utf-8




import rospy                                  
from std_msgs.msg import Float64                                                                   


def nodo_openbot():                                                                     
    
    rospy.init_node('nodo_move_base')         
           
                                                
    publisher_1 = rospy.Publisher('/openbot_v1/arm1_arm2_joint_position_controller/command',Float64, queue_size=1)
    publisher_2 = rospy.Publisher('/openbot_v1/clamp1_clamp2_joint_position_controller/command',Float64, queue_size=1)    
                                                                                         
    msg_1 = Float64()         #Definimos una variable de tipo Float64
    msg_2 = Float64()
    msg_1.data = 1.5   
    msg_2.data = 1.5   

    rate = rospy.Rate(10)     #Crea un objeto Rate a 10hz

    while not rospy.is_shutdown():              #Bucle While
        
        publisher_1.publish(msg_1)     
        rospy.sleep(1)                                               
        publisher_2.publish(msg_2)                    

if __name__ == '__main__':                                  
    try:
        nodo_openbot()                                 # Lamamos a la función nodo
    except rospy.ROSInterruptException :       # Check si hay una excepción  Ctrl-C para terminar la ejecución del nodo
            pass