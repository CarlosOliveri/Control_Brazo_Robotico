import numpy as np
import serial, time
from scipy.interpolate import CubicSpline

COM = 'COM5'
##DESCOMENTAR LA LINEA DE ABAJO PARA LA COMUNICACION SERIAL CON EL ARDUINO
#arduino = serial.Serial(COM,9600)
#time.sleep(2)

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

def cartesian2polar(x,y,z):
    r = np.sqrt(x**2 + y**2 + z**2)
    Q = np.arctan2(y,x)
    phi = np.arccos(z/r) 
    return r,Q,phi

def polar2cartesian(r,q,phi):
    x = r*np.sin(phi)*np.cos(q)
    y = r*np.sin(phi)*np.sin(q)
    z = r*np.cos(phi)
    return x, y, z

def Interpolacion(points):
    print(points)
    # Extraer los puntos individuales
    x_points = points[:, 0]
    y_points = points[:, 1]
    z_points = points[:, 2]

    # Crear una secuencia de valores t para los puntos originales
    t_points = np.linspace(0, 1, len(points))

    # Crear una secuencia de valores t para la interpolación (30 puntos)
    t_interpolated = np.linspace(0, 1, 30)

    # Realizar la interpolación cúbica
    cs_x = CubicSpline(t_points, x_points,bc_type = 'clamped')
    cs_y = CubicSpline(t_points, y_points,bc_type = 'clamped')
    cs_z = CubicSpline(t_points, z_points,bc_type = 'clamped')

    # Generar los puntos interpolados
    x_interpolated = cs_x(t_interpolated)
    y_interpolated = cs_y(t_interpolated)
    z_interpolated = cs_z(t_interpolated)

    # Guardar los puntos interpolados en una matriz
    interpolated_points = np.vstack((x_interpolated, y_interpolated, z_interpolated)).T

    for k in interpolated_points:
        k[2] = k[2] - 20.2
        r,q,phi = cartesian2polar(k[0], k[1], k[2]) #Max = 40.415 Min = 25.224 
        print(r,q,phi)
        if phi > - 49:
            phi = 48
        if r < 26:
            r = 26
            x, y, z = polar2cartesian(r,q,phi)
            k [0] = x
            k [1] = y
            k [2] = z + 20.1
        if r > 40:
            r = 40
            x, y, z = polar2cartesian(r,q,phi)
            k [0] = x
            k [1] = y
            k [2] = z + 20.1
        
        print("Puntos de la interpolacion:")
        print(interpolated_points)
        return interpolated_points

def Envio_mensaje(msg):
    #Arriba hay que descomentar la definicion del objeto arduino
    #Arduino en tiempo de debugger es NONE
    #arduino.write(msg.encode())
    pass

def Envio_Interpolacion(array):
    cont = 0
    #avisoInicio = arduino.readline().decode().strip()
    #print(avisoInicio)
    msg = "STR"
    for k in array:
        q1 = k[0]
        q2 = k[1]
        q3 = k[2]
        cont += 1
        respondio = False
        if np.isnan(q2):
            q2 = 0
        if np.isnan(q3):
            q3 = 0
        msg = msg + " " + str(q1) + " " + str(q2) + " " +str(q3)
    Envio_mensaje(msg)
    print("[Mensaje enviado] => " + msg)
    """ while respondio == False:
        time.sleep(2) #simulacion
        response = "Listo"
        #response = arduino.readline().decode().strip()
        print(response)
        if response == "Listo":
            respondio = True """

        

while(True):
    #STR XA YA ZA XB YB ZB XC YC ZC XD YD ZD XE YE ZE XF YF ZF
    msg = input("[Escriba el comando] => ")
    msg = msg.strip()
    msg = msg.split()
    try:
        for k in range(1,len(msg)):
            msg[k] = float(msg[k])
        funcion = msg[0]
    except:
        print("Mensaje Vacio")
        continue
    
    type = 1 # Valor por defecto
    
    if funcion == "P1":
        if len(msg) < 8 or len(msg) > 8:
            print("No se incluyeron todos los parametros: [G# x y x a b g type]")
        else:
            type = msg[-1]
            pr = msg[1:4]
            orient = msg[4:7]
            try:
                q1, q2, q3, q4, q5, q6 = inverse_kinematics_P1(pr[0], pr[1], pr[2], orient[0], orient[1], orient[2])  
                msg =  funcion +" "+ str(q1) + " " + str(q2) + " " + str(q3) + " " + str(q4) + " " + str(q5) + " " + str(q6) + " " + str(type)         
                print("[Mensaje enviado] => " + msg)
                Envio_mensaje(msg)
            except:
                print("Valores no validos, Verifique y vuelva a intentar!")
        continue
    
    if funcion == "S00":
        if len(msg) < 1 or len(msg) > 1:
            print("No se incluyeron todos los parametros")
        else:
            msg = funcion
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
        continue
    
    if funcion == "G00":
        if len(msg) < 1 or len(msg) > 1:
            print("No se incluyeron todos los parametros")
        else:
            msg = funcion
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
        continue
    if funcion == "G1":
        if len(msg) < 5 or len(msg) > 5:
            print("No se incluyeron todos los parametros: [G# x y z type]")
        else:
            type = msg[-1]
            pm = msg[1:4]
            print(pm)
            try: 
                q1, q2, q3 = inverse_kinematics_G1(pm[0], pm[1], pm[2])
                msg =  funcion +" "+ str(q1) + " " + str(q2) + " " + str(q3) + " " + str(type)
                #msg = str(q1) + "," + str(q2) + "," + str(q3) + "," + str(type)         
                print("[Mensaje enviado] => " + msg)
                Envio_mensaje(msg)
            except:
                print("Valores no validos, Verifique y vuelva a intentar!")
        continue
    if funcion == "G2":
        q = msg[1]
        alfa = msg[2]
        velocidad = msg[-1]
        if velocidad <= 0 or velocidad > 20:
            print("Valor de velocidad fuera de rango!, valores aceptados: [ 0 < vel <= 20 ]")
        else:
            if len(msg) < 4 or len(msg) > 4:
                print("Cantidad de parametros incorrecto: [G# # alpha vel]")
            else:
                if  q != 1:
                    if abs(alfa) > 90:
                        print("EL ANGULO ESTA FUERA DE RANGO")
                        continue 
                msg =  funcion +" "+ str(q) + " " + str(alfa) + " " + str(velocidad)         
                print("[Mensaje enviado] => " + msg)
                Envio_mensaje(msg)
        continue
    
    if funcion == "G13":
        q = msg[1]
        alfa = msg[2]
        velocidad = msg[3]
        beta1 = msg[4]
        beta2 = msg[5]
        if velocidad <= 0 or velocidad > 20:
            print("Valor de velocidad fuera de rango!, valores aceptados: [ 0 < vel <= 20 ]")
        if  q != 1:
            if abs(alfa) > 90 or abs(beta1) > 90 or abs(beta2) > 90:
                print("UNO DE LOS ANGULOS ESTA FUERA DE RANGO")
                continue
        msg = funcion + " " + str(q) + " " + str(alfa) + " " + str(velocidad) + " " + str(beta1) + " " + str(beta2)
        print("[Mensaje enviado] => " + msg)
        Envio_mensaje(msg)
        continue
    if funcion == "STR":
        points = np.array([ [0, 0, 0],        # Punto 1
                            [0, 0, 0],  # Punto 2
                            [0, 0, 0],  # Punto 3
                            [0, 0, 0],  # Punto 4
                            [0, 0, 0],
                            [0, 0, 0]])
        points[0,0:3] = msg[1:4]
        points[1,0:3] = msg[4:7]
        points[2,0:3] = msg[7:10]
        points[3,0:3] = msg[10:13]
        points[4,0:3] = msg[13:16]
        points[5,0:3] = msg[16:19]
        
        #Verificar si los puntos estan en el espacio de la tarea
        arrayQs = []
        interpolated_points = Interpolacion(points)
        try:
            for k in interpolated_points:
                q1, q2, q3 = inverse_kinematics_G1(k[0],k[1],k[2])
                arrayQs.append([q1,q2,q3])
            Envio_Interpolacion(arrayQs)
        except:
            print("UNO DE LOS PUNTOS DE LA INTERPOLACION ESTA FUERA DEL RANGO")
        continue
    if funcion == "GTR":
        if len(msg) > 1 or len(msg) < 1:
            print("Cantidad de parametros incorrecto")
        else:
            msg = funcion
            print("[Mensaje enviado] => " + msg)
            Envio_mensaje(msg)
        continue
    print("NO CORRESPONDE A NINGUN COMANDO")
    
    ## DESCOMENTAR LAS TRES LINEAS DE ABAJO PARA VERIFICAR LA RECEPCION DEL MENSAJE   
    #arduino.write(msg.encode())
    #respuesta = arduino.readline().decode().strip()
    #print("[Mensaje Recibido] => " + respuesta)}
    
#Funciones
# STR 15 15 35 21.7 13.2 37.7 23.9 12.9 43.5 10.2 14.5 44.2 -8 16.5 45.9 -12.4 15.7 45.7  Movimiento en S en el espacio
# STR 15 25 28.2 0 25 43.2 -15 25 28.2 0 25 13.2 15 25 28.2 0 25 43.2 Movimiendo de circulo en el plano Y
#G1 12 15 45.2
#G1 13 -15 404
#G1 -15 12 45
#G1 13 16 46
#G2 1 
#P1 26.215 0 36.2 0 -90 0 1