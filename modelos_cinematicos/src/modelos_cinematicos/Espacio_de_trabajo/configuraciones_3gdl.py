#!/usr/bin/env python
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import String


class configuraciones(object):
    def callback(data):

        if data.data == "1":
            configuraciones.desmarcar_workspace()
            #print("Desactivè workspace")

        if data.data == "2":
            configuraciones.desmarcar_borrar_workspace()
            #print("Limpìe workspace")
        else:
            pass

    def menu():
        rospy.init_node("dynamic_client")
        rospy.Subscriber('/configuration_rqt', String, configuraciones.callback)
        rospy.spin()

    
    def desmarcar_workspace():
        r = rospy.Rate(0.1)
        client = dynamic_reconfigure.client.Client("node_workspace_3gdl",timeout=30)
        workspace_params = {"Workspace" : False }
        client.update_configuration(workspace_params)
        r.sleep()

    def desmarcar_borrar_workspace():
        r = rospy.Rate(0.1)
        client = dynamic_reconfigure.client.Client("node_workspace_3gdl",timeout=30)
        workspace_params = {"Borrar_workspace" : False }
        client.update_configuration(workspace_params)
        r.sleep()

    def desmarcar_workspace():
        r = rospy.Rate(0.1)
        client = dynamic_reconfigure.client.Client("node_workspace_3gdl",timeout=30)
        workspace_params = {"Workspace" : False }
        client.update_configuration(workspace_params)
        r.sleep()

if __name__=="__main__":
    configuraciones.menu()