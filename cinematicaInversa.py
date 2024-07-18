import numpy as np
import serial, time

COM = 'COM5'
##DESCOMENTAR LA LINEA DE ABAJO PARA LA COMUNICACION SERIAL CON EL ARDUINO
#arduino = serial.Serial(COM,9600)
time.sleep(2)

l1 = 20.2 #cm
l2 = 16 #cm
l3 = 19.5 #cm
l4 = 6.715 #cm
d = np.array([0, 0, l4])

A01 = lambda q1: np.array([ [np.cos(q1),    0, np.sin(q1),     0],
                            [np.sin(q1),    0, -np.cos(q1),    0],
                            [     0,        1,       0,       l1],
                            [     0,        0,       0,        1]])
A12 = lambda q2: np.array([ [-np.sin(q2), -np.cos(q2),   0,  -l2*np.sin(q2)],
                            [np.cos(q2), -np.sin(q2),    0,  l2*np.cos(q2)],
                            [       0,                  0,              1,                      0],
                            [       0,                  0,              0,                      1]])
A23 = lambda q3: np.array([ [np.sin(q3), 0, np.cos(q3),    0],
                            [-np.cos(q3), 0,  np.sin(q3),    0],
                            [     0,            -1,       0,  0],
                            [     0,             0,       0,  1]])
A34 = lambda q4: np.array([ [np.cos(q4),    0,  np.sin(q4), 0],
                            [np.sin(q4),    0, -np.cos(q4), 0],
                            [     0,        1,       0,    l3],
                            [     0,        0,       0,     1]])
A45 = lambda q5: np.array([ [np.cos(q5),    0,  -np.sin(q5), 0],
                            [np.sin(q5),    0, np.cos(q5), 0],
                            [     0,                -1,          0,          0],
                            [     0,                0,          0,          1]])
A56 = lambda q6: np.array([ [np.cos(q6), -np.sin(q6), 0,        0],
                            [np.sin(q6),  np.cos(q6), 0,        0],
                            [     0,         0,       1,       l4],
                            [     0,         0,       0,        1]])
T = lambda phi,theta,thi: np.array([[np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta)*np.sin(thi)-np.sin(phi)*np.sin(thi), np.cos(phi)*np.sin(theta)*np.cos(thi)+np.sin(phi)*np.sin(thi), 0],
                                    [np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta)*np.sin(thi)+np.cos(phi)*np.cos(thi), np.sin(phi)*np.sin(theta)*np.cos(thi)-np.cos(phi)*np.sin(thi), 0],
                                    [       -np.sin(theta),                 np.cos(theta)*np.sin(thi),                              np.cos(theta)*np.cos(thi),                                0],
                                    [             0,                                     0,                                                      0,                                           1]])
#phi = pmz = alfa
# theta = pmy = beta
# thi = pmx = gama

def direct_kinematics(q1,q2,q3,q4,q5,q6):
    _0A2 = np.dot(A01(q1),A12(q2))
    _0A3 = np.dot(_0A2,A23(q3))
    _0A4 = np.dot(_0A3,A34(q4))
    _0A5 = np.dot(_0A4,A45(q5))
    _0A6 = np.dot(_0A5,A56(q6))
    return _0A6     

def inverse_kinematics_P1(x, y,z, a, b, g):
    signoQ3=-1
    codo = 1
    
    #Calculo del vector de orientacion del eje de la herramienta con respecto al eje de la base Angulos de Euler
    alpha=a*np.pi/180.0
    beta=b*np.pi/180.0
    gamma=g*np.pi/180.0
    Transform = T(alpha, beta, gamma) #Matripmz de tranformacion directa
    
    n = Transform[0,0:3]
    o = Transform[1,0:3]
    a = Transform[2,0:3]
    
    #Calculo de punto muñeca
    pmx = x - l4*a[0]
    pmy = y - l4*a[1]
    pmz = z - l4*a[2] -l1
    print("Punto Muñeca Calculado: ")       
    print(pmx,pmy,pmz) #debug
    
    # Halla q1
    q1 = np.arctan2(pmy,pmx)
    if q1 > np.pi or q1 < -np.pi:
        print("Fuera de rango")
        return None
            
    #Hallar q3
    cosq3=(pmx**2+pmy**2+pmz**2-l2**2-l3**2)/(2*l2*l3)
    sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
    q3=np.arctan2(sinq3,cosq3)
    if q3 > np.pi/1.9 or q3 < -np.pi/1.9:
        signoQ3*=-1
        sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
        q3=np.arctan2(sinq3,cosq3)
        if q3 > np.pi/1.9 or q3 < -np.pi/1.9:
            print("fuera del rango")
            return None
    #Hallar q2
    q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
    if q2 > np.pi/1.9 or q2 < -np.pi/1.9:
        codo*=-1
        q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
        if q2 > np.pi/1.9 or q2 < -np.pi/1.9:
            print("fuera del rango")
            return None
        
    cosq1=np.cos(q1)
    cosq2=np.cos(q2)
    sinq1=np.sin(q1)
    sinq2=np.sin(q2) 
    sinq3=np.sin(q3) 
    
    """ A03 = np.dot(A01(q1),A12(q2))# los angulos aca deben estar en radianes
    A03 = np.dot(A03,A23(q3))
    #print(A03)#debug
    R03_inv = A03[0:3,0:3].T
    #print(R03_inv)#debug
    R36 = np.dot(R03_inv,Transform[0:3,0:3]) 
    
    q5 = -np.arccos(R36[2,2]) 
    q4 = np.arccos(R36[1,2]/np.sin(q5))
    q6 = -np.arcsin(R36[2,1]/np.sin(q5))
    q4 = np.degrees(q4)
    q5 = np.degrees(q5)
    q6 = np.degrees(q6) """
    
    q5=np.arccos(-a[0]*cosq1*(cosq2*sinq3+cosq3*sinq2)-a[1]*sinq1*(cosq2*sinq3+cosq3*sinq2)+a[2]*(cosq2*cosq3-sinq2*sinq3))
    if q5 > np.pi/2 or q5 < -np.pi/2:
        print("Q5 fuera del rango")
        return None
    sinq5=np.sin(q5)

    q4=np.arctan2((cosq1*a[1]-sinq1*a[0]),abs(a[2]*(cosq2*sinq3+cosq3*sinq2)+(cosq2*cosq3-sinq2*sinq3)*(a[1]*sinq1+a[0]*cosq1)))

    sinq4=np.sin(q4)
    cosq4=np.cos(q4)

    q6=np.arcsin(n[0]*(-2*cosq4*sinq1+cosq1*sinq4*(-cosq2*cosq3+sinq2*sinq3))+n[1]*(cosq1*cosq4+sinq1*sinq4*(sinq2*sinq3-cosq2*cosq3))-n[2]*sinq4*(cosq2*sinq3+cosq3*sinq2))
    
    # Convertir a grados
    q1 = np.degrees(q1).round(decimals=3)
    q2 = np.degrees(q2).round(decimals=3)
    q3 = np.degrees(q3).round(decimals=3)
    q4 = np.degrees(q4).round(decimals=3)
    q5 = np.degrees(q5).round(decimals=3)
    q6 = np.degrees(q6).round(decimals=3)
    #print(str(q1) + str(q2) + str(q3) + str(q4) + str(q5) + str(q6))
   
    return q1, q2, q3, q4, q5, q6

def inverse_kinematics_G1(x, y, z):
    signoQ3=-1
    codo = 1

    #Calculo de punto muñeca
    pmx = x
    pmy = y
    pmz = z - l1
        
    print("Punto Muñeca Calculado: ")       
    print(pmx,pmy,pmz) #debug
    
    # Halla q1
    q1 = np.arctan2(pmy,pmx)
    if q1 > np.pi or q1 < -np.pi:
        print("fuera del rango")
        return None
    
    #q3
    cosq3=(pmx**2+pmy**2+pmz**2-l2**2-l3**2)/(2*l2*l3)
    sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
    q3=np.arctan2(sinq3,cosq3)
    if q3 > np.pi/1.9 or q3 < -np.pi/1.9:
        signoQ3*=-1
        sinq3=(signoQ3)*np.sqrt(1-cosq3**2)
        q3=np.arctan2(sinq3,cosq3)
        if q3 > np.pi/1.9 or q3 < -np.pi/1.9:
            print("fuera del rango")
            return None
    #print(q3)
    
    #q2
    q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
    if q2 > np.pi/1.9 or q2 < -np.pi/1.9:
        codo*=-1
        q2=(np.arctan2(pmz,(codo*np.sqrt(pmx**2+pmy**2))) - np.arctan2((l3*sinq3),(l2+l3*cosq3)))-np.pi/2
        if q2 > np.pi/1.9 or q2 < -np.pi/1.9:
            print("fuera del rango")
            return None
    #print(q2)
    
    q1 = np.degrees(q1).round(decimals=3)
    q2 = np.degrees(q2).round(decimals=3)
    q3 = np.degrees(q3).round(decimals=3)
    print("q1: ", str(q1))
    print("q2: ", str(q2))
    print("q3: ", str(q3))

    return q1, q2, q3

while(True):
    msg = input("[Escriba el comando] => ")
    msg = msg.strip()
    msg = msg.split()
    for k in range(1,len(msg) - 1):
        msg[k] = float(msg[k])
    funcion = msg[0]
    type = msg[-1]

    if funcion == "P1":
        if len(msg) < 8 or len(msg) > 8:
            print("No se incluyeron todos los parametros: [G# x y x a b g type]")
        else:
            pr = msg[1:4]
            orient = msg[4:7]
            try:
                q1, q2, q3, q4, q5, q6 = inverse_kinematics_P1(pr[0], pr[1], pr[2], orient[0], orient[1], orient[2])  
                msg =  funcion +" "+ str(q1) + " " + str(q2) + " " + str(q3) + " " + str(q4) + " " + str(q5) + " " + str(q6) + " " + type         
                print("[Mensaje enviado] => " + msg)
            except:
                print("Valores no validos, Verifique y vuelva a intentar!")
    if funcion == "G1":
        if len(msg) < 5 or len(msg) > 5:
            print("No se incluyeron todos los parametros: [G# x y z type]")
        else:
            pm = msg[1:4]
            print(pm)
            try: 
                q1, q2, q3 = inverse_kinematics_G1(pm[0], pm[1], pm[2])
                msg =  funcion +" "+ str(q1) + " " + str(q2) + " " + str(q3) + " " + type         
                print("[Mensaje enviado] => " + msg)
            except:
                print("Valores no validos, Verifique y vuelva a intentar!")
    if funcion == "G2":
        velocidad = msg[-1]
        if velocidad <= 0 or velocidad > 20:
            print("Valor de velocidad fuera de rango!, valores aceptados: [ 0 < vel <= 20 ]")
        else:
            print(msg)
    
    ## DESCOMENTAR LAS TRES LINEAS DE ABAJO PARA VERIFICAR LA RECEPCION DEL MENSAJE   
    #arduino.write(msg.encode())
    #respuesta = arduino.readline().decode().strip()
    #print("[Mensaje Recibido] => " + respuesta)}
