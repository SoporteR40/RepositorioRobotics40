#! /usr/bin/env python

import rospy

# Computing 
from math import acos,atan,sqrt,pi,cos,sin
from numpy import array

# Message 
from std_msgs.msg import Float64
from openbot_inverse_kinematic.msg import angle

# Server
from dynamic_reconfigure.server import Server


# Configuration 
from openbot_inverse_kinematic.cfg import Angle_jointConfig as config_angle

# TF
import tf2_ros
import tf2_msgs.msg


class Inverse_kinematic_compute():			
	def Calcul_angle2GDL_ab(x,z,L1=0.094,L2=0.262):
		"""Computing the two angles to get the final link in the position (x,z) (LOWER ELBOW)"""

		r=sqrt(x*x+z*z)
		value=((L1*L1+L2*L2-r*r)/(2*L1*r))
		if r*r<2*L1*L2+L1*L1+L2*L2 and value<1 and value>-1:					# To avoid math error 
			if x > 0:   # X positive
				q1=atan(z/x) - acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2))
			elif x==0:	# when x=0 arctan(z/x)-> pi/2
				q1=pi/2 - acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2))
			elif x<0:	# X negative
				q1= -(atan(z/-x) - acos((L1*L1+r*r-L2*L2)/(2*L1*r))) + pi
				q2= -(pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
			if q1< 0 or q1 > pi or q2 > 2.09 or q2 < -2.09:# To avoid overflow for the angle
				rospy.loginfo("Error the value are over the limit of the angle/joint x: %f",x)
			if q1< 0:
				q1=0
			if q1 > pi:
				q1=pi
			if q2 > 2.09:
				q2=2.09
			if q2 < -2.09:
				q2=-2.09
			result = array([pi/2-q1,-q2])										# The command for the 2 joint 
			rospy.loginfo(result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0])

	def Calcul_angle2GDL_arr(x,z,L1=0.094,L2=0.262):
		"""Computing the two angles to get the final link in the position (x,z) (UPPER ELBOW)"""

		r=sqrt(x*x+z*z)
		value=((L1*L1+L2*L2-r*r)/(2*L1*r))
		if r*r<2*L1*L2+L1*L1+L2*L2 and value<1 and value>-1 :					# To avoid math error 
			if x > 0:   # X positive
				q1=atan(z/x) + acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= -(pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
			elif x==0:	# when x=0 arctan(z/x)-> pi/2
				q1=pi/2 + acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= -(pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
			elif x<0:	# X negative
				q1= -(atan(z/-x) + acos((L1*L1+r*r-L2*L2)/(2*L1*r))) + pi
				q2= (pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
			
			if q1< 0 or q1 > pi or q2 > 2.09 or q2 < -2.09:# To avoid overflow for the angle
				rospy.loginfo("Error the value are over the limit of the angle/joint x: %f",x)
			if q1< 0:
				q1=0
			if q1 > pi:
				q1=pi
			if q2 > 2.09:
				q2=2.09
			if q2 < -2.09:
				q2=-2.09
			result = array([pi/2-q1,-q2]) 			# The command for the 2 joint 
			rospy.loginfo(result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0])
	
	def Calcul_angle3GDL_ab(x1,z1,qy,L1=0.094,L2=0.112,L3=0.15):
		"""Computing the three angles to get the final link in the position (x,z,qy) (LOWER ELBOW)"""
		

		if x1 >= 0:   # X positive
			if qy >= 0  and qy <= pi/2:
				x=x1 - L3*cos(qy)
				z=z1 - L3*sin(qy)
				rospy.loginfo("1")
			elif qy > pi/2  and qy <= pi  : 
				x=x1 + L3*cos(pi-qy)
				z=z1 - L3*sin(pi-qy)
				rospy.loginfo("2")
			elif qy >= -pi/2  and qy < 0: 
				x=x1 - L3*cos(qy)
				z=z1 + L3*sin(-qy)
				rospy.loginfo("4")
		
			elif (qy > -pi and qy < -pi/2) or qy > pi: 
				x=x1 + L3*cos(pi+qy)
				z=z1 - L3*sin(qy)
				rospy.loginfo("3")
		elif x1<0:
			if qy >= 0  and qy <= pi/2:
				x=x1 + L3*cos(qy)
				z=z1 - L3*sin(qy)
				rospy.loginfo("Cuadrante 1")
			elif qy > pi/2  and qy <= pi  : 
				x=x1 - L3*cos(pi-qy)
				z=z1 - L3*sin(pi-qy)
				rospy.loginfo("2")
			elif qy >= -pi/2  and qy < 0: 
				x=x1 + L3*cos(qy)
				z=z1 + L3*sin(-qy)
				rospy.loginfo("4")
		
			elif (qy > -pi and qy < -pi/2) or qy > pi: 
				x=x1 + L3*cos(pi+qy)
				z=z1 + L3*sin(pi+qy)
				rospy.loginfo("3")
				
				


		r=sqrt(x*x+z*z)
		value=((2*L1*L1+L2*L2-r*r)/(2*L1*r))
		if r*r<2*L1*L2+L1*L1+L2*L2 and value<1 and value>-1:					# To avoid math error 
			if x1 > 0:   # X positive
				q1=atan(z/x) - acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2))
				q3=qy-q1-q2
			elif x1==0:	# when x=0 arctan(z/x)-> pi/2
				q1=pi/2 - acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2))
				q3=qy-q1-q2
			elif x1<0:	# X negative
				q1= -(atan(z/-x) - acos((L1*L1+r*r-L2*L2)/(2*L1*r))) + pi
				print("Q1: ",q1)
				q2= -(pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
				q3=-(qy+q1-pi+q2)
			if (qy > -pi and qy < -pi/2):
				rospy.loginfo("Before :%f",q3)
				q3=(2*pi+q3)
				rospy.loginfo("After :%f",q3)
			rospy.loginfo([q1,q2,q3])
			if q1< 0.0 or q1 > pi or q2 > 2.09 or q2 < -2.09 or q3 > 2.61 or q3 < -1.57:    # To avoid overflow for the angle
				rospy.loginfo("Error the value are over the limit of the angle/joint ")
			if q1< 0.0:
				q1=0.0
			if q1 > pi:
				q1=pi
			if q2 > 2.09:
				q2=2.09
			if q2 < -2.09:
				q2=-2.09
			if q3 > 2.61:
				q3=2.61
			if q3 < -1.57:
				q3=-1.57
			result = array([pi/2-q1,-q2,-q3])                    # The command of the angle
			print("Valor de xp:",x)
			print("Valor de Zp:",z)
			q1g=q1*180/pi
			q2g=q2*180/pi
			q3g=q3*180/pi
			print('Codo Abajo')
			print('Q1 vale en grados: ',q1g)
			print('Q2 vale en grados  ',q2g)
			print('Q3 vale en grados: ',q3g)
			print('El resultado en radianes es: ',result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0,0])


	def Calcul_angle3GDL_arr(x1,z1,qy,L1=0.094,L2=0.112,L3=0.15):
		"""Computing the three angles to get the final link in the position (x,z,qy) (UPPER ELBOW)"""
		
		if x1 >= 0:   # X positive
			if qy >= 0  and qy <= pi/2:
				x=x1 - L3*cos(qy)
				z=z1 - L3*sin(qy)
				rospy.loginfo("1")
			elif qy > pi/2  and qy <= pi  : 
				x=x1 + L3*cos(pi-qy)
				z=z1 - L3*sin(pi-qy)
				rospy.loginfo("2")
			elif qy >= -pi/2  and qy < 0: 
				x=x1 - L3*cos(qy)
				z=z1 + L3*sin(-qy)
				rospy.loginfo("4")
		
			elif (qy > -pi and qy < -pi/2) or qy > pi: 
				x=x1 + L3*cos(pi+qy)
				z=z1 + L3*sin(pi+qy)
				rospy.loginfo("3")
		elif x1<0:
			if qy >= 0  and qy <= pi/2:
				x=x1 + L3*cos(qy)
				z=z1 - L3*sin(qy)
				rospy.loginfo("1")
			elif qy > pi/2  and qy <= pi  : 
				x=x1 - L3*cos(pi-qy)
				z=z1 - L3*sin(pi-qy)
				rospy.loginfo("2")
			elif qy >= -pi/2  and qy < 0: 
				x=x1 + L3*cos(qy)
				z=z1 + L3*sin(-qy)
				rospy.loginfo("4")
		
			elif (qy > -pi and qy < -pi/2) or qy > pi: 
				x=x1 + L3*cos(pi+qy)
				z=z1 + L3*sin(pi+qy)
				rospy.loginfo("3")

		
		r=sqrt(x*x+z*z)
		value=((L1*L1+L2*L2-r*r)/(2*L1*r))
		if r*r<2*L1*L2+L1*L1+L2*L2 and value<1 and value>-1:					# To avoid math error 
			if x1 > 0:   # X positive
				q1=atan(z/x) + acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= -(pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
				q3=qy-q1-q2
			elif x1==0:	# when x=0 arctan(z/x)-> pi/2
				q1=pi/2 + acos((L1*L1+r*r-L2*L2)/(2*L1*r))
				q2= -(pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
				q3=qy-q1-q2
			elif x1<0:	# X negative
				q1= -(atan(z/-x) + acos((L1*L1+r*r-L2*L2)/(2*L1*r))) + pi
				q2= (pi - acos((L1*L1+L2*L2-r*r)/(2*L1*L2)))
				q3=-(qy+q1-pi+q2)
			rospy.loginfo([pi/2-q1,-q2,-q3])
			if q1< 0.0 or q1 > pi or q2 > 2.09 or q2 < -2.09 or q3 > 2.61 or q3 < -1.57:    # To avoid overflow for the angle
				rospy.loginfo("Error the value are over the limit of the angle/joint ")
			if q1< 0.0:
				q1=0.0
			if q1 > pi:
				q1=pi
			if q2 > 2.09:
				q2=2.09
			if q2 < -2.09:
				q2=-2.09
			if q3 > 2.61:
				q3=2.61
			if q3 < -1.57:
				q3=-1.57
			result = array([pi/2-q1,-q2,-q3])
			print("Valor de xp:",x)
			print("Valor de Zp:",z)
			q1g=q1*180/pi
			q2g=q2*180/pi
			q3g=q3*180/pi
			print('Codo Abajo')
			print('Q1 vale en grados: ',q1g)
			print('Q2 vale en grados  ',q2g)
			print('Q3 vale en grados: ',q3g)
			print('El resultado en radianes es: ',result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0,0])
	
	
	def Calcul_angle3GDL_3Dimension_ab(x,y,z,L1=0.032,L2=0.094,L3=0.262):
		"""Computing the three angles to get the final link in the position (x,y,z) (LOWER ELBOW)"""
		
		
		r=sqrt(x*x+y*y+z*z)
		value=((L2*L2+r*r-L3*L3)/(2*L2*r))
		if r*r<2*L3*L2+L3*L3+L2*L2  and value<1 and value>-1:					# To avoid math error 
			if x > 0:   # X positive
				q1=atan(y/x)
				q2=atan(z/x) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
				q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
			elif x==0:	# when x=0 arctan(z/x)-> pi/2
				if y>0: # Y positive
					q1=pi/2
					q2= atan(z/y) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				elif y<0: # Y negative
					q1=-pi/2
					q2= atan(z/-y) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				else: #Y=0
					q1=0
					q2= pi/2 - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
			elif x<0:	# X negative
				if y>=0: # Y positive
					q1= -atan(y/-x)
				elif y<0: # Y negative
					q1= -atan(-y/-x)+pi/2
				q2= -(atan(z/-x) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))) + pi
				q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
			if q1> 2.61 or q1 < -2.61 or q2< 0.0 or q2 > pi  or -q3 > 2.09 or -q3 < -2.09:   #To avoid the overflow
				rospy.loginfo("Error the value are over the limit of the angle/joint ")
			if q2< 0:
				q2=0
				rospy.loginfo("Error q2")
			if q2 > pi:
				q2=pi
				rospy.loginfo("Error q2")
			if q3 > 2.09:
				q3=2.09
				rospy.loginfo("Error q3")
			if q3 < -2.09:
				rospy.loginfo("Error q3")
				q3=-2.09
			
			result = array([q1,pi/2-q2,-q3])
			
			rospy.loginfo(result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0,0])
	
	def Calcul_angle3GDL_3Dimension_arr(x,y,z,L1=0.032,L2=0.094,L3=0.262):
		"""Computing the three angles to get the final link in the position (x,y,z) (UPPER ELBOW)"""
		r=sqrt(x*x+y*y+z*z)
		value=((L2*L2+r*r-L3*L3)/(2*L2*r))
		if r*r<2*L3*L2+L3*L3+L2*L2  and value<1 and value>-1:					# To avoid math error 
			if x > 0:   # X positive
				q1=atan(y/x)
				q2=atan(z/x) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
				q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
			elif x==0:	# when x=0 arctan(z/x)-> pi/2
				if y>0: # Y positive
					q1=pi/2
					q2= atan(z/y) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				elif y<0: # Y negative
					q1=-pi/2
					q2= atan(z/-y) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				else: # Y=0
					q1=0
					q2= pi/2 + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
			elif x<0:	# X negative
				if y>=0: # Y positive
					q1= -atan(y/-x)
				elif y<0: # Y negative
					q1= -atan(-y/-x)+pi/2
				q2= -(atan(z/-x) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))) + pi
				q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
		
			if q1> 2.61 or q1 < -2.61 or q2< 0.0 or q2 > pi  or -q3 > 2.09 or -q3 < -2.09:   #To avoid the overflow
				rospy.loginfo("Error the value are over the limit of the angle/joint ")
			if q2< 0:
				q2=0
				rospy.loginfo("Error q2")
			if q2 > pi:
				q2=pi
				rospy.loginfo("Error q2")
			if q3 > 2.09:
				q3=2.09
				rospy.loginfo("Error q3")
			if q3 < -2.09:
				rospy.loginfo("Error q3")
				q3=-2.09
			
			result = array([q1,pi/2-q2,-q3])
			rospy.loginfo(result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0,0])
	
	
	def Calcul_angle4GDL_ab(x1,y1,z1,qy,L1=0.033,L2=0.094,L3=0.112,L4=0.15):
		"""Computing the fourth angles to get the final link in the position (x,y,z,qy) (LOWER ELBOW)"""
		#print("Codo abajo",x1,y1,z1,qy)
		if qy >= 0  and qy <= pi/2:   # (1)
			m=L4*cos(qy)
			z=z1-L4*sin(qy)
		elif qy>pi/2 and qy <= pi:     # (2)
			m=L4*cos(qy)
			z=z1-L4*sin(qy)
		elif qy >= -pi/2  and qy < 0:   # (4)
			m=L4*cos(qy)
			z=z1 + L4*sin(-qy)
		elif qy >= -pi  and qy < -pi/2:   # (3)
			m=L4*cos(qy)
			z=z1 - L4*sin(qy)
			
		if x1 > 0:   # X positive
			if y1>=0:
				q1=atan(y1/x1)
				y=y1-m*sin(q1)
				x=x1-m*cos(q1)
			else : #Y negative
				q1=atan(y1/x1)
				y=y1-m*sin(q1)
				x=x1-m*cos(q1)
		elif x1<0:	# X negative
			if y1>=0:
				q1=-atan(y1/-x1)
				y=y1-m*sin(-q1)
				x=x1+m*cos(q1)
			else : #Y positive
				q1=pi/2 - atan(y1/x1)
				y=y1-m*sin(-q1)
				x=x1+m*cos(q1)
		else: # X1=0 arctan(y1/x1) -> pi/2
			if y1>0: #Y positive
				q1=pi/2
			elif y1<0 : #Y negative
				q1=-pi/2
			else: #Y=0
				q1=0
			y=y1-m*sin(q1)
			x=x1-m*cos(q1)

		
		
		r=sqrt(x*x+y*y+z*z)
		value=((L2*L2+r*r-L3*L3)/(2*L2*r))
		if r*r<2*L3*L2+L3*L3+L2*L2 and value<1 and value>-1:					# To avoid math error 
			if x1 > 0:   # X positive
				q2=atan(z/x) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
				q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				q4=qy-q2-q3
			elif x1==0:	# when x=0 arctan(z/x)-> pi/2
				if y1>0: # Y positive
					q2= atan(z/y) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
					q4=qy-q2-q3
				elif y1<0: # Y negative
					q2= atan(z/-y) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
					q4=qy-q2-q3
				else:	# Y=0
					q2= pi/2 - acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
					q4=qy-q2-q3
			elif x1<0:	# X negative
				q2= -(atan(z/-x) - acos((L2*L2+r*r-L3*L3)/(2*L2*r))) + pi
				q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				q4=-(qy+q3-pi+q2)
				
			
			if q1> 2.61 or q1 < -2.61 or q2 > pi  or q2 < 0.0 or q3 > 2.09 or q3 < -2.09 or q4 > 2.6 or q4 < -1.57:   #To avoid the overflow
				rospy.loginfo("Error the value are over the limit of the angle")
			if q2< 0.0:
				q2=0.0
				rospy.loginfo("Error q2")
			if q2 >pi  :
				q2=pi
				rospy.loginfo("Error q2")
			if q3 < -2.09:
				q3=-2.09
				rospy.loginfo("Error q3")
			if q3 > 2.09:
				q3=2.09
				rospy.loginfo("Error q3")
			if q4 < -1.57:
				q4=-1.57
				rospy.loginfo("Error q4")
			if q4 > 2.61:
				q4=2.61
				rospy.loginfo("Error q4")
			
			result = array([q1,pi/2-q2,-q3,-q4])
			rospy.loginfo(result)
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0,0,0])
	
	def Calcul_angle4GDL_arr(x1,y1,z1,qy,L1=0.033,L2=0.094,L3=0.112,L4=0.15):
		"""Computing the fourth angles to get the final link in the position (x,y,z,qy) (UPPER ELBOW)"""
		#print("Codo arriba",x1,y1,z1,qy)
		if qy >= 0  and qy <= pi/2:
			m=L4*cos(qy)
			z=z1-L4*sin(qy)
		elif qy>pi/2 and qy <= pi:
			m=L4*cos(qy)
			z=z1-L4*sin(qy)
		elif qy >= -pi/2  and qy < 0: 
			m=L4*cos(qy)
			z=z1 + L4*sin(-qy)
		elif qy >= -pi  and qy < -pi/2: 
			m=L4*cos(qy)
			z=z1 -L4*sin(qy)
		if x1 > 0:   # X positive
			if y1>=0:
				q1=atan(y1/x1)
				y=y1-m*sin(q1)
				x=x1-m*cos(q1)
			else :
				q1=atan(y1/x1)
				y=y1-m*sin(q1)
				x=x1-m*cos(q1)
		
		elif x1<0:	# X negative
			if y1>=0:
				q1=-atan(y1/-x1)
				y=y1-m*sin(-q1)
				x=x1+m*cos(q1)
			else :
				q1=pi/2 - atan(y1/x1)
				y=y1-m*sin(-q1)
				x=x1+m*cos(q1)
		else: # X1=0 arctan(y1/x1) -> pi/2
			if y1>0:
				q1=pi/2
			elif y1<0 :
				q1=-pi/2
			else:
				q1=0
			y=y1-m*sin(q1)
			x=x1-m*cos(q1)
		
		r=sqrt(x*x+y*y+z*z)
		value=((L2*L2+r*r-L3*L3)/(2*L2*r))
		if r*r<2*L3*L2+L3*L3+L2*L2 and value<1 and value>-1 and x!=0:					# To avoid math error 
			if x1 > 0:   # X positive
				q2=atan(z/x) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
				q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				q4=qy-q2-q3
			elif x1==0:	# when x=0 arctan(z/x)-> pi/2
				if y1>0: # Y positive
					q2= atan(z/y) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
					q4=qy-q2-q3
				elif y1<0: # Y negative
					q2= atan(z/-y) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
					q4=qy-q2-q3
				else: # Y=0
					q2= pi/2 + acos((L2*L2+r*r-L3*L3)/(2*L2*r))
					q3= -(pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
					q4=qy-q2-q3
			elif x1<0:	# X negative
				q2= -(atan(z/-x) + acos((L2*L2+r*r-L3*L3)/(2*L2*r))) + pi
				q3= (pi - acos((L2*L2+L3*L3-r*r)/(2*L2*L3)))
				q4=-(qy+q3-pi+q2)


			if q1 > 2.61 or q1 < -2.61 or q2 > 3.47  or q2 < -0.42 or q3 > 2.09 or q3 < -2.09 or q4 > 2.6 or q4 < -1.57:   #To avoid the overflow
				rospy.loginfo("Error the value are over the limit of the angle")
			if q2< -0.42:
				#q2=-0.42
				rospy.loginfo("Error q2 :%f ",q2)
			if q2 >3.47  :
				#q2=3.47
				rospy.loginfo("Error q2 :%f ",q2)
			if q3 < -2.09:
				q3=-2.09
				rospy.loginfo("Error q3")
			if q3 > 2.09:
				q3=2.09
				rospy.loginfo("Error q3")
			if q4 < -1.57:
				q4=-1.57
				rospy.loginfo("Error q4 :%f ",q4)
			if q4 > 2.61:
				q4=2.61
				rospy.loginfo("Error q4 :%f ",q4)
			
			result = array([q1,pi/2-q2,-q3,-q4])
			return result
		else:
			rospy.loginfo("Wrong value !! The robot go to the init position...")
			return array([0,0,0,0])
