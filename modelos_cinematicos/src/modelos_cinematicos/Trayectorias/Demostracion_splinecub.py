import numpy as np
from math import trunc
import matplotlib.pyplot as plt


#Condiciones de ejemplo
Po=0.1
Pf= 0.2
Tf=10
n=100

dt=(Tf-0)/(n-1)
t=np.arange(0.0, Tf+0.00001, dt)


C=[ [0,0,0,1],
[0,0,1,0],
[Tf**3 , Tf**2 ,Tf,1],
[3*(Tf**2), 2*Tf,  1, 0] 
]

B= [[Po], [0], [Pf], [0]]

A= np.dot(np.linalg.inv(C),B)

a=A[0]
b=A[1]
c=A[2]
d=A[3]
t_3= t**3
t_2=t**2

P=a*t_3+b*t_2+c*t+d
dp=3*a*t_2+2*b*t+c
ddp=6*a*t+2*b



plt.subplot(3, 1, 1)
plt.plot(t, P,'r')
plt.title("Gràfica de Posiciòn", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'})
plt.xlabel('time (s)')
plt.ylabel('Posiciòn')
plt.grid(True)


plt.subplot(3, 1, 2)
plt.plot(t, dp, 'r')
plt.title("Gràfica de Velocidad", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'})
plt.xlabel('time (s)')
plt.ylabel('Velocidad')
plt.grid(True)


plt.subplot(3, 1, 3)
plt.title("Gràfica de Aceleraciòn", loc="center", fontdict={'fontsize':14,'fontweight':'bold', 'color':'tab:blue'})
plt.plot(t,ddp, 'r')
plt.xlabel('time (s)')
plt.ylabel('Aceleraciòn')
plt.grid(True)

#plt.subplot_tool()
plt.subplots_adjust(left=0.12,bottom=0.11,right=0.9,top=0.88,wspace=0.2,hspace=0.58)
plt.show()