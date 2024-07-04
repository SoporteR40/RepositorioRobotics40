
from modelos_cinematicos.Cinematica_Inversa.cinematica_inversa_calculos import calculos_cinematica_inversa
from modelos_cinematicos.Cinematica_Directa.calculos_cinematica_directa import calculos_cinematica_directa


###Còdigo para comprobar Cinematica Directa 2g
#Datos de entrada
x=-0.18
z=0.24
qy=120
L1=0.094
L2=0.112
L3=0.15
print("Coordenadas Iniciales son : (",x,",",z,")" )
codo=int(input("¿codo abajo?(1) o ¿Codo arriba?(2)   "))


#Funciòn de calculos de Cinematica Inversa
if codo == 1:
    result=calculos_cinematica_inversa.cal_3GDL_aba(x,z,qy,L1,L2,L3)
if codo ==  2:
    result=calculos_cinematica_inversa.cal_3GDL_arri(x,z,qy,L1,L2,L3)

print(result)
#Funciòn de calculos de CInematica Directa
if codo == 1:
    result_d=calculos_cinematica_directa.cal_3GDL_abajo(result[0],result[1],result[2],L1,L2,L3)
if codo ==  2:
    result_d=calculos_cinematica_directa.cal_3GDL_arriba(result[0],result[1],result[2],L1,L2,L3)

print("¿Las coordenadas iniciales eran?: (",result_d[0],",",result_d[1],")" )


