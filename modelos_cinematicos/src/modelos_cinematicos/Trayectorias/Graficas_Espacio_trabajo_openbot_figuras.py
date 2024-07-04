import numpy as np
from math import acos,atan,sqrt,pi,cos,asin,sin
from modelos_cinematicos.Cinematica_Directa.calculos_cinematica_directa import calculos_cinematica_directa
from modelos_cinematicos.Trayectorias.splinecub_puntos import splinecub
import matplotlib.pyplot as plt

class Graficas_Espacio_de_trabajo_figuras():

    def grafica_workspace_2g_abajo_figuras(self,figura):
        fig, ax = plt.subplots()
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

        ax.set_title("Espacio de Trabajo 2GDL Codo Abajo", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'}) 
        ax.plot(x, z,color= 'tab:red')
        ax.plot(x1, z1,color= 'tab:red')
        ax.plot(x2, z2,color= 'tab:red')
        ax.plot(xf, zf,color= 'tab:red')

        if figura=="1":

            ###Parametros de entrada
            corrimientox=-0.01
            corriemientoz=0.25
            base=-0.06
            altura=0.08
            n=100
            tf=10

            #Rango de X
            x1=splinecub(corrimientox,corrimientox+base,10,n/4)
            x2=splinecub(corrimientox+base,corrimientox+base,10,n/4)
            x3=splinecub(corrimientox+base,corrimientox,10,n/4)
            x4=splinecub(corrimientox,corrimientox,10,n/4)
            #Rango de z
            z1=splinecub(altura+corriemientoz,altura+corriemientoz,10,n/4)
            z2=splinecub(altura+corriemientoz,corriemientoz,10,n/4)
            z3=splinecub(corriemientoz,corriemientoz,10,n/4)
            z4=splinecub(corriemientoz,corriemientoz+altura,10,n/4)

            #Z=[z_pos,z_neg]
            ax.plot(x1, z1,color= 'tab:blue')
            ax.plot(x2, z2,color= 'tab:blue')
            ax.plot(x3, z3,color= 'tab:blue')
            ax.plot(x4, z4,color= 'tab:blue')

        elif figura=="2":
            n=100
            tf=10
            ###Parametros de entrada
            limiteiz=-0.01
            limiteinf=0.25
            base=-0.09
            altura=0.07
            

            limitede=limiteiz+base
            x1=splinecub(limiteiz,(limitede+limiteiz)/2,10,n/3)
            x2=splinecub((limiteiz+limitede)/2,limiteiz+base,10,n/3)
            x3=splinecub(limiteiz+base,limiteiz,10,n/3)

            z1=splinecub(limiteinf,altura+limiteinf,10,n/3)
            z2=splinecub(altura+limiteinf,limiteinf,10,n/3)
            z3=splinecub(limiteinf,limiteinf,10,n/3)

            ax.plot(x1, z1,color= 'tab:blue')
            ax.plot(x2, z2,color= 'tab:blue')
            ax.plot(x3, z3,color= 'tab:blue')

        elif figura=="3":
            n=100
            tf=10
            radio =0.04
            yc=0.30 #Valor desplazar el centro del circulo en y
            xc=-0.025 #Valor desplazar el centro del circulo en x
            trayectoria_x= np.array(splinecub(-radio+xc,radio+xc,10,n/2))
            trayectoria_x2= np.array(splinecub(radio+xc,-radio+xc,10,n/2))

            #Rango de z
            z_pos=np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc
            z_neg=(-1*np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc)
            ax.plot(trayectoria_x, z_pos,color= 'tab:blue')
            ax.plot(trayectoria_x2, z_neg,color= 'tab:blue')

        else:
            print("No escogiste ninguna figura")

        plt.show()

        

    def grafica_workspace_2g_arriba_figuras(self,figura):
        fig, ax = plt.subplots()
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

        ax.set_title("Espacio de Trabajo 2GDL Codo Arriba", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'}) 
        ax.plot(x, z,color= 'tab:red')
        ax.plot(x1, z1,color= 'tab:red')
        ax.plot(x2, z2,color= 'tab:red')
        ax.plot(xf, zf,color= 'tab:red')

        if figura=="1":

            ###Parametros de entrada
            corrimientox=0.01
            corriemientoz=0.25
            base=0.06
            altura=0.06
            n=100
            tf=10

            #Rango de X
            x1=splinecub(corrimientox,corrimientox+base,10,n/4)
            x2=splinecub(corrimientox+base,corrimientox+base,10,n/4)
            x3=splinecub(corrimientox+base,corrimientox,10,n/4)
            x4=splinecub(corrimientox,corrimientox,10,n/4)
            #Rango de z
            z1=splinecub(altura+corriemientoz,altura+corriemientoz,10,n/4)
            z2=splinecub(altura+corriemientoz,corriemientoz,10,n/4)
            z3=splinecub(corriemientoz,corriemientoz,10,n/4)
            z4=splinecub(corriemientoz,corriemientoz+altura,10,n/4)

            #Z=[z_pos,z_neg]
            ax.plot(x1, z1,color= 'tab:blue')
            ax.plot(x2, z2,color= 'tab:blue')
            ax.plot(x3, z3,color= 'tab:blue')
            ax.plot(x4, z4,color= 'tab:blue')
        if figura=="2":
            n=100
            tf=10
            ###Parametros de entrada
            limiteiz=0.01
            limiteinf=0.25
            base=0.09
            altura=0.05

            limitede=limiteiz+base
            x1=splinecub(limiteiz,(limitede+limiteiz)/2,10,n/3)
            x2=splinecub((limiteiz+limitede)/2,limiteiz+base,10,n/3)
            x3=splinecub(limiteiz+base,limiteiz,10,n/3)

            z1=splinecub(limiteinf,altura+limiteinf,10,n/3)
            z2=splinecub(altura+limiteinf,limiteinf,10,n/3)
            z3=splinecub(limiteinf,limiteinf,10,n/3)

            ax.plot(x1, z1,color= 'tab:blue')
            ax.plot(x2, z2,color= 'tab:blue')
            ax.plot(x3, z3,color= 'tab:blue')

        if figura=="3":
            n=100
            tf=10
            radio =0.04
            yc=0.285 #Valor desplazar el centro del circulo en y
            xc=0.025 #Valor desplazar el centro del circulo en x

            trayectoria_x= np.array(splinecub(-radio+xc,radio+xc,10,n/2))
            trayectoria_x2= np.array(splinecub(radio+xc,-radio+xc,10,n/2))
            #Rango de z

            z_pos=np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc
            z_neg=(-1*np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc)
            ax.plot(trayectoria_x, z_pos,color= 'tab:blue')
            ax.plot(trayectoria_x2, z_neg,color= 'tab:blue')

        else:
            print("No escogiste ninguna figura")
        plt.show()

    def grafica_workspace_3g_qy_figuras(self,qy,figura):
        fig, ax = plt.subplots()
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
        
        ax.set_title("Espacio de Trabajo 3GDL respecto a un Qy", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'}) 
        ax.plot(x, z,color= 'tab:red')
        ax.plot(x1, z1,color= 'tab:red')
        ax.plot(x2, z2,color= 'tab:red')
        ax.plot(x3, z3,color= 'tab:red')
        ax.plot(x4, z4,color= 'tab:red')
        if figura=="1":
            ###Parametros de entrada
            corrimientox=0.06
            corriemientoz=0.145
            base=0.05
            altura=0.05
            n=100
            tf=10

            #Rango de X
            x1=splinecub(corrimientox,corrimientox+base,10,n/4)
            x2=splinecub(corrimientox+base,corrimientox+base,10,n/4)
            x3=splinecub(corrimientox+base,corrimientox,10,n/4)
            x4=splinecub(corrimientox,corrimientox,10,n/4)
            #Rango de z
            z1=splinecub(altura+corriemientoz,altura+corriemientoz,10,n/4)
            z2=splinecub(altura+corriemientoz,corriemientoz,10,n/4)
            z3=splinecub(corriemientoz,corriemientoz,10,n/4)
            z4=splinecub(corriemientoz,corriemientoz+altura,10,n/4)

            #Z=[z_pos,z_neg]
            ax.plot(x1, z1,color= 'tab:blue')
            ax.plot(x2, z2,color= 'tab:blue')
            ax.plot(x3, z3,color= 'tab:blue')
            ax.plot(x4, z4,color= 'tab:blue')

        if figura=="2":
            n=100
            tf=10
            ###Parametros de entrada
            limiteiz=0.06
            limiteinf=0.145
            base=0.06
            altura=0.06

            limitede=limiteiz+base
            x1=splinecub(limiteiz,(limitede+limiteiz)/2,10,n/3)
            x2=splinecub((limiteiz+limitede)/2,limiteiz+base,10,n/3)
            x3=splinecub(limiteiz+base,limiteiz,10,n/3)

            z1=splinecub(limiteinf,altura+limiteinf,10,n/3)
            z2=splinecub(altura+limiteinf,limiteinf,10,n/3)
            z3=splinecub(limiteinf,limiteinf,10,n/3)

            ax.plot(x1, z1,color= 'tab:blue')
            ax.plot(x2, z2,color= 'tab:blue')
            ax.plot(x3, z3,color= 'tab:blue')

        if figura=="3":
            n=100
            tf=10
            radio =0.028
            yc=0.17 #Valor desplazar el centro del circulo en y
            xc=0.08 #Valor desplazar el centro del circulo en x

            trayectoria_x= np.array(splinecub(-radio+xc,radio+xc,10,n/2))
            trayectoria_x2= np.array(splinecub(radio+xc,-radio+xc,10,n/2))
            #Rango de z

            z_pos=np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc
            z_neg=(-1*np.sqrt(radio**2-np.power(trayectoria_x-xc,2))+yc)
            ax.plot(trayectoria_x, z_pos,color= 'tab:blue')
            ax.plot(trayectoria_x2, z_neg,color= 'tab:blue')

        else:
            print("No escogiste ninguna figura")

        plt.show()