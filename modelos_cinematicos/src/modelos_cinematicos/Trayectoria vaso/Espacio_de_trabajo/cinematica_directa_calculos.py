#librerias de calculos
import numpy as np
from math import acos,atan,sqrt,pi,cos,asin,sin


class calculos_cinematica_directa():
    def cal_2GDL_abajo(q1,q2,L1,L2):
        #ROBOT CODO ABAJO
        #conversion de los angulos de grados a radianes
        q1 = (q1*pi)/180
        q2 = (q2*pi)/180

        # Condiciones para cada cuadrante

        # Primer cuadrante
        if(q1+q2 >=0 and q1+q2 <=(pi/2)):
            x = L1*cos(q1)+L2*cos(q1+q2)
            z= L1*sin(q1)+L2*sin(q1+q2)

        #segundo cuadrante
        elif(q1+q2 > pi/2 and q1+q2 <= pi):
            # angulo auxiliar
            q1p = q1 - pi
            
            x = -L1*cos(q1p)-L2*cos(q1p+q2)
            z = -L1*sin(q1p)-L2*sin(q1p+q2)

        #tercer cuadrante
        elif(q1+q2 > pi and q1+q2 <= ((3/2)*pi)):
            #angulo auxiliar
            q1p = q1 - pi
            x = -L1*cos(q1p)-L2*cos(q1p+q2)
            z = -L1*sin(q1p)-L2*sin(q1p+q2)
            
        #cuarto cuadrante
        elif(q1+q2> ((3/2)*pi) and q1+q2 <= (2*pi)):
            #angulo auxiliar
            q1p = q1 - (2*pi)
            x = L1*cos(q1p)+L2*cos(q1p+q2)
            z = L1*sin(q1p)+L2*sin(q1p+q2)

        print('El valor de x es: ',x)
        print('El valor de z es: ',z)
        return x,z

    #ROBOT CODO ARRIBA
    def cal_2GDL_arriba(q1,q2,L1,L2):
        
        #conversion de grados a radianes
        q1 = (q1*pi)/180
        q2 = (q2*pi)/180

        #condiciones para cada cuadrante

        #Primer cuadrante
        if(q1+q2 >= 0 and q1+q2 <= (pi/2)):
            q1p = q1-pi
            q2p = q1+q2
            
            x = -L1*cos(q1p)+L2*cos(q2p)
            z = -L1*sin(q1)+L2*sin(q2p)
        #segundo cuadrante
        elif (q1+q2 > pi/2 and q1+q2 <= pi):
            #angulo auxiliar
            q2p = q1 + q2 - pi
            
            x = L1*cos(q1)-L2*cos(q2p)
            z = L1*sin(q1)-L2*sin(q2p)
            
        #tercer cuadrante
        elif (q1+q2 > pi and q1+q2 <= ((3/2)*pi)):
            #angulos auxiliares
            q1p = q1 - (2*pi)
            q2p = q1 + q2 - pi
            
            x = L1*cos(q1p)-L2*cos(q2p)
            z = L1*sin(q1p)-L2*sin(q2p)

        #cuarto cuadrante
        elif (q1+q2 > ((3/2)*pi) and q1+q2 <= (2*pi)):
            #angulos auxiliares
            q1p = q1 - pi
            q2p = q1 + q2 - (2*pi)
            
            x = -L1*cos(q1p)+L2*cos(q2p)
            z = -L1*sin(q1p)+L2*sin(q2p)

        print('El valor de x es: ',x)
        print('El valor de z es: ',z)
        return x,z

    def cal_3GDL_arriba(q1,q2,q3,L1,L2,L3):
        #conversion de los angulos
        q1=(q1*pi)/180
        q2=(q2*pi)/180
        q3=(q3*pi)/180
        
        #Se halla el valor de qy
        qy=q1+q2+q3
        
        #condicionales para qy
        if ( (qy>=0 and qy<=pi/2) or (qy < -3*pi/2 and qy>= -2*pi) ): #Cuadrante 1
            x3=L3*cos(qy)
            z3=L3*sin(qy)
        elif ( (qy>pi/2 and qy<=pi) or ( qy<-pi and qy>= -3*pi/2)): #Cuadrante 2
            x3=-L3*cos(qy-pi)
            z3=-L3*sin(qy-pi)
        elif ((qy>pi and qy<=3*pi/2) or (qy< -pi/2 and qy >= -pi)): #Cuadrante 3
            x3=-L3*cos(qy-pi)
            z3=-L3*sin(qy-pi)
        elif ((qy>3*pi/2 and qy <= 2*pi) or (qy<=0 and qy>= -pi/2)): #Cuadrante 4
            x3=L3*cos(qy-(2*pi))
            z3=L3*sin(qy-(2*pi))
    
        #condiciones para cada cuadrante
        
        #Condicion para Primer cuadrante
        if ((q1+q2 >= 0 and q1+q2 <= pi/2) or (q1+q2<-3*pi/2 and q1+q2>=-2*pi)):
            q1p=q1-pi
            q2p=q1+q2
            x1=-L1*cos(q1p)
            z1=-L1*sin(q1p)
            x2=L2*cos(q2p)
            z2=L2*sin(q2p)
            
            x=x1+x2+x3
            z=z1+z2+z3
        #Condicion para Segundo cuadrante
        elif ((q1+q2 > pi/2 and q1+q2 <= pi) or (q1+q2<-pi and q1+q2>=-3*pi/2)):
            #Agregamos el desfase de q2
            
            q2p=q1+q2-pi
            
            x1=L1*cos(q1)
            z1=L1*sin(q1)
            x2=-L2*cos(q2p)
            z2=-L2*sin(q2p)
            
            x=x1+x2+x3
            z=z1+z2+z3
        # Condicion Tercer cuadrante
        elif ((q1+q2>pi and q1+q2<= 3/2*pi) or (q1+q2 < -pi/2 and q1+q2 >=-pi)):
            q1p= q1-(2*pi)
            q2p= q1+q2-180
            
            x1=L1*cos(q1p)
            z1=L1*sin(q1p)
            x2=-L2*cos(q2p)
            z2=-L2*sin(q2p)
            
            x=x1+x2+x3
            z=z1+z2+z3
        #Cuarto cuadrante
        elif ((q1+q2> 3*pi/2 and q1+q2<=2*pi) or (q1+q2<= 0 and q1+q2>=-pi/2)):
            q1p= q1-pi
            q2p=q1+q2-(2*pi)
            
            x1=-L1*cos(q1p)
            z1=-L1*sin(q1p)
            x2=L2*cos(q2p)
            z2=L2*sin(q2p)
            
            x=x1+x2+x3
            z=z1+z2+z3
        
        #print('El valor de x es: ',x)
        #print('El valor de z es: ',z)
        #print('El angulo qy es: ',qy)
        return x,z
    
    def cal_3GDL_abajo(q1,q2,q3,L1,L2,L3):
        #conversiones de los angulos
        q1 = (q1*pi)/180
        q2 = (q2*pi)/180
        q3 = (q3*pi)/180

        #Se determina el valor de qy

        qy = q1+q2+q3

        #Condicionales para qy

        if (qy >= 0 and qy <= pi/2):
            x3 = L3*cos(qy)
            z3 = L3*sin(qy)
        elif(qy > pi/2 and qy <= pi):
            x3 = -L3*cos(qy-pi)
            z3 = -L3*sin(qy-pi)
        elif(qy > pi and qy <= 3*pi/2):
            x3 = -L3*cos(qy-pi)
            z3 = -L3*sin(qy-pi)
        elif(qy > 3*pi/2 and qy <= 2*pi/2):
            x3 = L3*cos(qy-(2*pi))
            z3 = L3*sin(qy-(2*pi))


        #Condicionales para cada cuadrante

        #Cuadrante 1
        if(q1+q2 >= 0 and q1+q2 <= (pi/2)):

            x1 = L1*cos(q1)
            z1 = L1*sin(q1)
            x2 = L2*cos(q1+q2)
            z2 = L2*sin(q1+q2)

            x = x1+x2+x3
            z = z1+z2+z3
    
        #Cuadrante 2
        elif(q1+q2 > pi/2 and q1+q2 <= pi):

            q1p = q1-pi

            x1 = -L1*cos(q1p)
            z1 = -L1*sin(q1p)
            x2 = -L2*cos(q1p+q2)
            z2 = -L2*sin(q1p+q2)

            x = x1+x2+x3
            z = z1+z2+z3

        #cuadrante 3
        elif(q1+q2 > pi and q1+q2 <= ((3/2)*pi)):

            q1p = q1-pi

            x1 = -L1*cos(q1p)
            z1 = -L1*sin(q1p)
            x2 = -L2*cos(q1p+q2)
            z2 = -L2*sin(q1p+q2)

            x = x1+x2+x3
            z = z1+z2+z3
        #cuadrante 4
        elif(q1+q2 > ((3/2)*pi) and q1+q2 <= (2*pi)):

            q1p = q1-(2*pi)

            x1 = L1*cos(q1p)
            z1 = L1*sin(q1p)
            x2 = L2*cos(q1p+q2)
            z2 = L2*sin(q1p+q2)

            x = x1+x2+x3
            z = z1+z2+z3

        qyg=qy*180/pi
        print('El valor de x es: ',x)
        print('El valor de z es: ',z)
        print('El angulo qy es: ',qyg)
        
        return x,z





