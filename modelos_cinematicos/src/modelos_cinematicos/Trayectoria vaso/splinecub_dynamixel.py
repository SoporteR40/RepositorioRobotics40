import numpy as np
from math import trunc

def splinecub_2 (Po, Pf, Tf, n):

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
    P=P.astype(int)
    return P