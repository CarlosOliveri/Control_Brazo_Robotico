import numpy as np
import serial, time
l1 = 20.2 #cm
l2 = 16 #cm
l3 = 19.5 #cm
l4 = 6.715 #cm
d = np.array([0, 0, l4])

A01 = lambda q1: np.array([[np.cos(q1), 0, np.sin(q1),  0],
               [np.sin(q1), 0, -np.cos(q1), 0],
               [     0,         1,       0,        l1],
               [     0,         0,       0,         1]])
A12 = lambda q2: np.array([[np.cos(q2+np.pi/2), -np.sin(q2+np.pi/2), 0, l2*np.cos(q2+np.pi/2)],
                [np.sin(q2+np.pi/2), np.cos(q2+np.pi/2),  0, l2*np.sin(q2+np.pi/2)],
                [     0,               0,         1,                 0],
                [     0,               0,         0,                 1]])
A23 = lambda q3: np.array([[np.cos(q3-np.pi/2), 0, -np.sin(q3-np.pi/2), 0],
                [np.sin(q3-np.pi/2), 0,  np.cos(q3-np.pi/2), 0],
                [     0,        -1,       0,         0],
                [     0,         0,       0,         1]])
A34 = lambda q4: np.array([[np.cos(q4), 0,  np.sin(q4), 0],
                [np.sin(q4), 0, -np.cos(q4), 0],
                [     0,         1,       0,        l3],
                [     0,         0,       0,         1]])
A45 = lambda q5: np.array([[np.cos(q5+np.pi/2), 0,  np.sin(q5+np.pi/2), 0],
                [np.sin(q5+np.pi/2), 0, -np.cos(q5+np.pi/2), 0],
                [     0,         1,       0,         0],
                [     0,         0,       0,         1]])
A56 = lambda q6: np.array([[np.cos(q6), -np.sin(q6), 0, 0],
                [np.sin(q6),    np.cos(q6), 0, 0],
                [     0,         0,       1,       l4],
                [     0,         0,       0,        1]])
T = lambda phi,theta,thi: np.array([[np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta)*np.sin(thi)-np.sin(phi)*np.sin(thi), np.cos(phi)*np.sin(theta)*np.cos(thi)+np.sin(phi)*np.sin(thi), 0],
                                    [np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta)*np.sin(thi)+np.cos(phi)*np.cos(thi), np.sin(phi)*np.sin(theta)*np.cos(thi)-np.cos(phi)*np.sin(thi), 0],
                                    [       -np.sin(theta),                 np.cos(theta)*np.sin(thi),                              np.cos(theta)*np.cos(thi),                                0],
                                    [             0,                                     0,                                                      0,                                           1]])
#phi = z = alfa
# theta = y = beta
# thi = x = gama

def direct_kinematics(q1,q2,q3,q4,q5,q6):
    _0A2 = np.dot(A01(q1),A12(q2))
    _0A3 = np.dot(_0A2,A23(q3))
    _0A4 = np.dot(_0A3,A34(q4))
    _0A5 = np.dot(_0A4,A45(q5))
    _0A6 = np.dot(_0A5,A56(q6))
    return _0A6     

def inverse_kinematics(x, y,z):
    # Calcula theta3 usando la ley del coseno
    if x == 0:
        theta1_rad = np.pi/2
    else:
        theta1_rad = np.arctan(y/x)
    cosq3 = (x**2 + y**2 + (z-l1)**2 - l2**2 - l3**2) / (2*l2*l3)
    if abs(cosq3) > 1:
        raise TypeError("[NAN]")
    theta3_rad = np.arctan(np.sqrt(1-(cosq3)**2)/cosq3)
    u = np.sqrt(x**2 + y**2 + (z-l1)**2)
    theta2_rad =  np.pi/2 - (np.arctan((z-l1)/np.sqrt(x**2 + y**2)) + np.arccos((l2**2 + u**2 - l3**2)/(2*l2*u)))

    # Convertir a grados
    """ theta1_deg = np.degrees(theta1_rad)
    theta2_deg = np.degrees(theta2_rad)
    theta3_deg = np.degrees(theta3_rad) """
    print(theta2_rad,theta3_rad)
    #print(cosq3) # debug
    if abs(theta2_rad) > np.pi/2:
        raise Exception("[Fuera de rango]")
    if abs(theta3_rad) > np.pi/2:
        raise Exception("[Fuera de rango]")

    return theta1_rad, theta2_rad, theta3_rad

""" COM = 'COM5'
##DESCOMENTAR LA LINEA DE ABAJO PARA LA COMUNICACION SERIAL CON EL ARDUINO
#arduino = serial.Serial(COM,9600)
time.sleep(2)
 """
while(True):
    msg = input("[Escriba el comando] => ")
    msg = msg.strip()
    msg = msg.split()
    for k in range(0,len(msg)):
        msg[k] = float(msg[k])
    #print(msg) #debug
    pr = msg[0:3]
    orient = msg[3:6]
    #Calculo del vector de orientacion del eje de la herramienta con respecto al eje de la base
    Transform = T(float(orient[0])*np.pi/180, float(orient[1])*np.pi/180, float(orient[2])*np.pi/180) #Matriz de tranformacion directa
    #pm = pr 
    #print(Transform[0:3,2])
    pm = pr - Transform[0:3,2]*l4 #Calculo de punto muñeca
    #pm  =  [20, 18, 40] #Para probar directamente el punto muñeca
    #print(pm) #debug
    #print(d)  #debug
    try:
        theta1_rad, theta2_rad, theta3_rad = inverse_kinematics(float(pm[0]), float(pm[1]), float(pm[2]))
        #Conversion de angulos a decimal
        theta1_deg = np.degrees(theta1_rad)
        theta2_deg = np.degrees(theta2_rad)
        theta3_deg = np.degrees(theta3_rad)
        A03 = np.dot(A01(theta1_rad),A12(theta2_rad))# los angulos aca deben estar en radianes
        A03 = np.dot(A03,A23(theta3_rad))
        """ R03 = np.array([ #de chatgpt
        [np.cos(theta1_rad) * np.cos(theta2_rad + theta3_rad), -np.sin(theta1_rad), np.cos(theta1_rad) * np.sin(theta2_rad + theta3_rad)],
        [np.sin(theta1_rad) * np.cos(theta2_rad + theta3_rad), np.cos(theta1_rad), np.sin(theta1_rad) * np.sin(theta2_rad + theta3_rad)],
        [-np.sin(theta2_rad + theta3_rad), 0, np.cos(theta2_rad + theta3_rad)]]) """
        #print(A03)#debug
        R03_inv = A03[0:3,0:3].T
        #print(R03_inv)#debug
        R36 = np.dot(R03_inv,Transform[0:3,0:3])
        #R36 = np.dot(R03.T, Transform[0:2,0:2]) de chatgpt
        print(R36) #debug
        
        #Despejes para optener q4,q5,q6
        #cos(q5) = R36(2,2)
        #sen(q4) = R36(0,2)/sen(q5)
        #sen(q6) = R36(2,1)/sen(q5)   
        theta5_rad = -np.arccos(R36[2,2]) 
        theta4_rad = np.arccos(R36[1,2]/np.sin(theta5_rad))
        theta6_rad = -np.arcsin(R36[2,1]/np.sin(theta5_rad))
        theta4_deg = np.degrees(theta4_rad)
        theta5_deg = np.degrees(theta5_rad)
        theta6_deg = np.degrees(theta6_rad)
        msg =  str(theta1_deg) + " , " + str(theta2_deg) + " , " + str(theta3_deg) + " , " + str(theta4_deg) + " , " + str(theta5_deg) + " , " + str(theta6_deg) + " , " + str(msg[-1])        
        print("[Mensaje enviado] => " + msg) 
        print("Cinematica Directa:")
        print(direct_kinematics(theta1_rad,theta2_rad,theta3_rad,theta4_rad,theta5_rad,theta6_rad))
        #print(A03)
        ## DESCOMENTAR LAS TRES LINEAS DE ABAJO PARA VERIFICAR LA RECEPCION DEL MENSAJE   
        #arduino.write(msg.encode())
        #respuesta = arduino.readline().decode().strip()
        #print("[Mensaje Recibido] => " + respuesta)
         
    except TypeError:
        print("[NAN] Combinacion de posicion no valida")
    except ZeroDivisionError:
        print("[Division entre cero]")
    """ except Exception:
        print("[Fuera de Rango]") """
    