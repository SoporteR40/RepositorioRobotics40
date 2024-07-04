import numpy as np
from math import acos,atan,sqrt,pi,cos,asin,sin
from Espacio_de_trabajo.cinematica_directa_calculos import calculos_cinematica_directa
import matplotlib.pyplot as plt

def graficar_espacio_de_trabajo(vector_x,vector_z):
    #Dimensiones robot (Los unicos parametros que se cambian junto con el número de muestras)
    L1 = 0.095
    L2 = 0.115
    L3 = 0.13
    #Numero de muestras
    n=60
    #Definimos la gráfica
    fig, ax = plt.subplots()
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
    ###############Primer Limite
    dq1=-180/(n-1)
    q1=np.arange(180,-1,dq1)
    q2=np.zeros((n))
    q3=np.zeros((n))
    #Calculamos X y Z
    for i in range(n):
        x[i],z[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1[i],q2[i],q3[i],L1,L2,L3)

    #print("x:",x)
    #print("z:",z)

    ###############Segundo Limite
    q1p=np.ones(n)*180
    q2p=np.zeros((n))
    q3m=42
    dq3m=q3m/(n-1)
    q3p=np.arange(0,q3m+dq3m,dq3m)*-1

    for i in range(n):
        x1[i],z1[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1p[i],q2p[i],q3p[i],L1,L2,L3)

    #print("x1:",x1)
    #print("z1:",z1)


    ######Tercer Limite
    q1p2=np.ones(n)*180
    q2m=114
    dq2m=q2m/(n-1)
    q2p2=np.arange(0,q2m+dq2m,dq2m)*-1
    q3p2=np.ones(n)*-42

    for i in range(n):
        x2[i],z2[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1p2[i],q2p2[i],q3p2[i],L1,L2,L3)

    #print("x2:",x2)
    #print("z2:",z2)

    #Ultimo Limite (Teoremas del seno y coseno para hallar el ultimo valor máximo)
    teta=180-q2m
    La=sqrt(L1**2+L2**2-(2*L1*L2*cos(teta*pi/180)))
    q1p=asin((sin(teta*pi/180)/La)*L2) *180/pi
    beta=180-q1p-teta

    alpha=180-q3m-beta
    Lb=sqrt(La**2+L3**2-(2*La*L3*cos(alpha*pi/180)))
    q1c=asin((sin(alpha*pi/180)/Lb) *L3)*180/pi

    q1m=q1p+q1c

    dq1m=-(180-q1m)/(n-1)
    q1f=np.arange(180,q1m,dq1m)
    q2f=-114*np.ones(n)
    q3f=-42*np.ones(n)

    for i in range(n):
        x3[i],z3[i]= calculos_cinematica_directa.cal_3GDL_arriba(q1f[i],q2f[i],q3f[i],L1,L2,L3)

    #print("x3:",x3)
    #print("z3:",z3)
    #Cerrar el  espacio de trabajo:
    x2f=x3[n-1]
    x1f=x[n-1]
    dx=(x2f-x1f)/(n-1)
    XF=np.arange(x1f,x2f+dx,dx)
    ZF=np.zeros(n)
    #print("xf:",XF)
    #print("zf:",ZF)

    #Graficamos todo el espacio de trabajo
    ax.set_title("Espacio de Trabajo 3GDL Codo Arriba", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'}) 
    ax.plot(x, z,color= 'tab:red')
    ax.plot(x1,z1,color= 'tab:red')
    ax.plot(x2,z2,color= 'tab:red')
    ax.plot(x3,z3,color= 'tab:red')
    ax.plot(XF,ZF,color= 'tab:red')
    ax.plot(vector_x,vector_z,color= 'tab:green')
    for i in range(len(vector_x)):
        ax.scatter(vector_x[i],vector_z[i],color= 'tab:blue')

    plt.grid()
    plt.show()

