import numpy as np
from math import acos,atan,sqrt,pi,cos,asin,sin
from modelos_cinematicos.Cinematica_Directa.calculos_cinematica_directa import calculos_cinematica_directa

class Espacio_de_trabajo():

    def workspace_2g_abajo(self):
        #Dimensiones
        L1=0.094
        L2=0.262
        #Numero de muestras
        n=60
        #Inicializacion de vectores:
        x=np.zeros((n))
        z=np.zeros((n))
        x1=np.zeros((n))
        z1=np.zeros((n))
        x2=np.zeros((n))
        z2=np.zeros((n))


        #Primer limite o limite exterior
        q1=np.linspace(180,0,num=n,endpoint=True)
        q2= np.zeros(n)

        for i in range(n):
            x[i],z[i]= calculos_cinematica_directa.cal_2GDL_abajo(q1[i],q2[i],L1,L2,)


        max_q2=120
        q2=np.linspace(0,max_q2,num=n,endpoint=True)
        q1= np.zeros(n)


        for i in range(n):
            x1[i],z1[i]= calculos_cinematica_directa.cal_2GDL_abajo(q1[i],q2[i],L1,L2,)


        teta=180-max_q2
        La=sqrt(L1*L1+L2*L2-(2*L1*L2*cos(teta*pi/180)))
        qx=acos((La*La+L1*L1-L2*L2)/(2*L1*La))*180/pi
        q1_max=180-qx

        q1=np.linspace(0,q1_max,num=n,endpoint=True)
        q2=120*(np.ones(n))


        for i in range(n):
            x2[i],z2[i]= calculos_cinematica_directa.cal_2GDL_abajo(q1[i],q2[i],L1,L2,)


        x1f=x2[-1]
        x2f=x[0]
        xf=np.linspace(x2f,x1f,num=n,endpoint=True)
        zf=np.zeros(n)

        result=np.round([[x,z],[x1,z1],[x2,z2],[xf,zf]],4)
        return result

    def workspace_2g_arriba(self):
        #Dimensiones
        L1=0.094
        L2=0.262
        #Numero de muestrcas
        n=60
        #Inicializacion de vectores:
        x=np.zeros((n))
        z=np.zeros((n))
        x1=np.zeros((n))
        z1=np.zeros((n))
        x2=np.zeros((n))
        z2=np.zeros((n))


        #Primer limite o limite exterior
        q1=np.linspace(0,180,num=n,endpoint=True)
        q2= np.zeros((n))

        for i in range(n):
            x[i],z[i]= calculos_cinematica_directa.cal_2GDL_arriba(q1[i],q2[i],L1,L2,)



        q1= np.ones(n)*180
        max_q2=-120
        q2=np.linspace(0,max_q2,num=n,endpoint=True)

        for i in range(n):
            x1[i],z1[i]= calculos_cinematica_directa.cal_2GDL_arriba(q1[i],q2[i],L1,L2,)


        teta=180+max_q2
        La=sqrt(L1*L1+L2*L2-(2*L1*L2*cos(teta*pi/180)))
        qx=acos((La*La+L1*L1-L2*L2)/(2*L1*La))*180/pi
        q1_max= qx
        q1=np.linspace(180,q1_max,num=n,endpoint=True)
        q2=-120*(np.ones(n))


        for i in range(n):
            x2[i],z2[i]= calculos_cinematica_directa.cal_2GDL_arriba(q1[i],q2[i],L1,L2,)

        x1f=x2[-1]
        x2f=x[0]
        xf=np.linspace(x2f,x1f,num=n,endpoint=True)
        zf=np.zeros(n)

        result=[[x,z],[x1,z1],[x2,z2],[xf,zf]]
        return result
        
    def workspace_3g_abajo(self):
        #Dimensiones robot (Los unicos parametros que se cambian junto con el número de muestras)
        L1 = 0.094
        L2 = 0.112
        L3 = 0.15
        #Numero de muestras
        n=60
        #Definimos los vectores
        x=np.zeros((n))
        z=np.zeros((n))
        x1=np.zeros((n))
        z1=np.zeros((n))
        x2=np.zeros((n))
        z2=np.zeros((n))
        x3=np.zeros((n))
        z3=np.zeros((n))
        ##################################################Obtencion del espacio de trabajo
        ###########Primer Limite

        q1=np.linspace(180,0,num=n,endpoint=True)
        q2=np.zeros((n))
        q3=np.zeros((n))

        #Calculamos X y Z
        for i in range(n):
            x[i],z[i]= calculos_cinematica_directa.cal_3GDL_abajo(q1[i],q2[i],q3[i],L1,L2,L3)



        ##############Segundo Limite
        q1p=np.zeros((n))
        q2p=np.zeros((n))
        q3m=90
        q3p=np.linspace(0,q3m,num=n,endpoint=True)

        for i in range(n):
            x1[i],z1[i]= calculos_cinematica_directa.cal_3GDL_abajo(q1p[i],q2p[i],q3p[i],L1,L2,L3)


        ##########Tercer Limite
        q1p2=np.zeros((n))
        q2m=120
        q2p2=np.linspace(0,q2m,num=n,endpoint=True)
        q3p2=np.ones(n)*q3m

        for i in range(n):
            x2[i],z2[i]= calculos_cinematica_directa.cal_3GDL_abajo(q1p2[i],q2p2[i],q3p2[i],L1,L2,L3)

        #Ultimo Limite (Teoremas del seno y coseno para hallar el ultimo valor máximo)
        teta=180-q2m
        La=sqrt(L1**2+L2**2-(2*L1*L2*cos(teta*pi/180)))
        q1p=acos((L1*L1+La*La-L2*L2)/(2*L1*La))*180/pi
        beta=180-q1p-teta
        alpha=180-q3m-beta
        Lb=sqrt(La**2+L3**2-(2*La*L3*cos(alpha*pi/180)))
        q1c=acos((La*La+Lb*Lb-L3*L3)/(2*La*Lb))*180/pi

        q1m=180-q1p-q1c
        q1f=np.linspace(0,q1m,num=n,endpoint=True)
        q2f=q2m*np.ones(n)
        q3f=q3m*np.ones(n)

        for i in range(n):
            x3[i],z3[i]= calculos_cinematica_directa.cal_3GDL_abajo(q1f[i],q2f[i],q3f[i],L1,L2,L3)

        #Cerrando el espacio de trabajo
        x1f = x3[-1]           
        x2f = x[0]
        z1f = z3[-1]           
        z2f = z[0]               

        XF=np.linspace(x1f,x2f,num=n,endpoint=True)
        ZF =np.linspace(z1f,z2f,num=n,endpoint=True) 
        
        result=np.round([[x,z],[x1,z1],[x2,z2],[x3,z3],[XF,ZF]],4)
        return result

    def workspace_3g_arriba(self):

        #Dimensiones robot (Los unicos parametros que se cambian junto con el número de muestras)
        L1 = 0.094
        L2 = 0.112
        L3 = 0.15
        #Numero de muestras
        n=60
        #Definimos los vectores
        vector_x=[]
        vector_z=[]
        x=np.zeros((n))
        z=np.zeros((n))
        x1=np.zeros((n))
        z1=np.zeros((n))
        x2=np.zeros((n))
        z2=np.zeros((n))
        x3=np.zeros((n))
        z3=np.zeros((n))
        ##################################################Obtencion del espacio de trabajo
        ###############Primer Limite

        q1=np.linspace(0,180,num=n,endpoint=True)
        q2=np.zeros((n))
        q3=np.zeros((n))

        #Calculamos X y Z
        for i in range(n):
            x[i],z[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1[i],q2[i],q3[i],L1,L2,L3)
            vector_x.append(x[i])
            vector_z.append(z[i])

        ###############Segundo Limite
        q1p=np.ones(n)*180
        q2p=np.zeros((n))
        q3m=-90
        q3p=np.linspace(0,q3m,num=n,endpoint=True)


        for i in range(n):
            x1[i],z1[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1p[i],q2p[i],q3p[i],L1,L2,L3)
            vector_x.append(x1[i])
            vector_z.append(z1[i])


        ######Tercer Limite
        q1p2=np.ones(n)*180
        q2m=-120
        q2p2=np.linspace(0,q2m,num=n,endpoint=True)
        q3p2=np.ones(n)*q3m

        for i in range(n):
            x2[i],z2[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1p2[i],q2p2[i],q3p2[i],L1,L2,L3)
            vector_x.append(x2[i])
            vector_z.append(z2[i])

        #Ultimo Limite (Teoremas del seno y coseno para hallar el ultimo valor máximo)
        teta=180+q2m
        La=sqrt(L1**2+L2**2-(2*L1*L2*cos(teta*pi/180)))
        q1p=acos((L1*L1+La*La-L2*L2)/(2*L1*La))*180/pi
        beta=180-q1p-teta

        alpha=180+q3m-beta
        Lb=sqrt(La**2+L3**2-(2*La*L3*cos(alpha*pi/180)))
        q1c=acos((La*La+Lb*Lb-L3*L3)/(2*La*Lb))*180/pi

        q1m=q1p+q1c

        q1f=np.linspace(180,q1m,num=n,endpoint=True)
        q2f=q2m*np.ones(n)
        q3f=q3m*np.ones(n)

        for i in range(n):
            x3[i],z3[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1f[i],q2f[i],q3f[i],L1,L2,L3)
            vector_x.append(x3[i])
            vector_z.append(z3[i])


        #Cerrar el  espacio de trabajo:
        x1f = x3[-1]           
        x2f = x[0]
        z1f = z3[-1]           
        z2f = z[0]  
        XF=np.linspace(x1f,x2f,num=n,endpoint=True)
        ZF =np.linspace(z1f,z2f,num=n,endpoint=True)      
        
        
        result=np.round([[x,z],[x1,z1],[x2,z2],[x3,z3],[XF,ZF]],4)
        return result
    def workspace_3g_qy(self,qy):

        #Reajuste de qy:
        if qy<=90:
            qy=90-qy
        elif qy>90:
            qy=(qy-90)*-1

        #Dimensiones robot (Los unicos parametros que se cambian junto con el número de muestras)
        L1 = 0.094
        L2 = 0.112
        L3 = 0.15
        #Numero de muestras
        V=60
        #Definimos los vectores
        x=np.zeros((V))
        z=np.zeros((V))
        x1=np.zeros((V))
        z1=np.zeros((V))
        x2=np.zeros((V))
        z2=np.zeros((V))
        x3=np.zeros((V))
        z3=np.zeros((V))
        x4=np.zeros((V))
        z4=np.zeros((V))
        ##################################################Obtencion del espacio de trabajo

        if qy<0 :
            phi=-qy
            maximoArt4 = 90
            minArt4 = -90
        else:
            phi=qy
            maximoArt4 = 120
            minArt4 = -90

        maximoArt2 = 90 # maximo valor articulacion 2
        maximoArt3 = 120# maximo valor articulacion 3


        maxArt3 = phi+maximoArt2-maximoArt4;# maximo del siguiente movimiento depende de phi(para phi=90 este da 30)
        minArt3 = phi-maximoArt2;# minimo del siguiente movimiento depende de phi(para phi=90 este da 90)
        ###########Primer trazo

        q1 = np.linspace(90-phi,180,num=V,endpoint=True) 
        q2=np.linspace(0,-maxArt3,num=V,endpoint=True)
        q3=np.linspace(0,-maximoArt4,num=V,endpoint=True)

        #Calculamos X y Z
        for i in range(V):
            x[i],z[i]= calculos_cinematica_directa.cal_3GDL_qy(q1[i],q2[i],q3[i],L1,L2,L3)

        #Segundo trazo
        aux = phi+maximoArt2-maximoArt3
        q1= 180*np.ones(V)
        q2 = np.linspace(-maxArt3,-maximoArt3,num=V,endpoint=True) 
        q3 = np.linspace(-maximoArt4,-aux,num=V,endpoint=True) 

        #Calculamos X y Z
        for i in range(V):
            x1[i],z1[i]= calculos_cinematica_directa.cal_3GDL_qy(q1[i],q2[i],q3[i],L1,L2,L3)

        #Tercer trazo:
        maxArt2 = phi-minArt4-maximoArt3
        q1 =np.linspace(180,90-maxArt2,num=V,endpoint=True)  
        q2 =-maximoArt3*np.ones(V)
        q3 = np.linspace(-aux,-minArt4,num=V,endpoint=True) 

        #Calculamos X y Z
        for i in range(V):
            x2[i],z2[i]= calculos_cinematica_directa.cal_3GDL_qy(q1[i],q2[i],q3[i],L1,L2,L3)


        #Cuarto trazo
        maxArt3 = phi-minArt4-90

        q1 =  np.linspace(90-maxArt2,0,num=V,endpoint=True)
        q2 = np.linspace(-maximoArt3,-maxArt3,num=V,endpoint=True)
        q3 = -minArt4*np.ones(V)

        #Calculamos X y Z
        for i in range(V):
            x3[i],z3[i]= calculos_cinematica_directa.cal_3GDL_qy(q1[i],q2[i],q3[i],L1,L2,L3)


        #Quintro trazo:
        q1 =  np.linspace(0,90-phi,num=V,endpoint=True)
        q2 =  np.linspace(-maxArt3,0,num=V,endpoint=True)
        q3 =  np.linspace(-minArt4,0,num=V,endpoint=True)

        #Calculamos X y Z
        for i in range(V):
            x4[i],z4[i]= calculos_cinematica_directa.cal_3GDL_qy(q1[i],q2[i],q3[i],L1,L2,L3)

        if qy<0:
            x=-x
            x1=-x1
            x2=-x2
            x3=-x3
            x4=-x4
        
        result=np.round([[x,z],[x1,z1],[x2,z2],[x3,z3],[x4,z4]],4)

        return result