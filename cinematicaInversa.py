import numpy as np
import serial, time
l1 = 20.2 #cm
l2 = 16 #cm
l3 = 19.5 #cm
l4 = 6.715 #cm
d = np.array([l4, 0, 0])

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
""" A34 = lambda q4: np.array([[np.cos(q4), 0,  np.sin(q4), 0],
                [np.sin(q4), 0, -np.cos(q4), 0],
                [     0,         1,       0,        l3],
                [     0,         0,       0,         1]])
A45 = lambda q5: np.array([[np.cos(q5+np.pi/2), 0,  np.sin(q5+np.pi/2), 0],
                [np.sin(q5+np.pi/2), 0, -np.cos(q5+np.pi/2), 0],
                [     0,         1,       0,         0],
                [     0,         0,       0,         1]])
A56 = lambda q6: np.array([[np.cos(q6), np.sin(q6), 0, 0],
                [np.sin(q6), np.cos(q6), 0, 0],
                [     0,         0,       1,       l4],
                [     0,         0,       0,        1]]) """
T = lambda phi,theta,thi: np.array([[np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta)*np.cos(thi)+np.sin(phi)*np.sin(thi), np.cos(phi)*np.sin(theta)*np.sin(thi)+np.sin(phi)*np.cos(thi), 0],
                           [np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta)*np.sin(thi)+np.cos(phi)*np.cos(thi), np.sin(phi)*np.sin(theta)*np.cos(thi)+np.cos(phi)*np.sin(thi), 0],
                           [-np.sin(theta), np.cos(theta)*np.sin(thi), np.cos(theta)*np.cos(thi), 0],
                           [0, 0, 0, 1]])

def multMatrizial(A,B):
    prod = []
    fila = []
    for k in range(0,4):
        a = np.dot(A[k,0:4],B[0:4,0])
        b = np.dot(A[k,0:4],B[0:4,1])
        c = np.dot(A[k,0:4],B[0:4,2])
        d = np.dot(A[k,0:4],B[0:4,3])
        fila.append(a)
        fila.append(b)
        fila.append(c)
        fila.append(d)
        prod.append(fila)
        fila = []
    prod = np.array(prod)
    return prod

def inverse_kinematics(x, y,z):
    # Calcula theta3 usando la ley del coseno
    if x == 0:
        theta1_rad = np.pi/2
    else:
        theta1_rad = np.arctan(y/x)
    cosq3 = (x**2 + y**2 + z**2 - l2**2 - l3**2) / (2*l2*l3)
    if abs(cosq3) > 1:
        raise TypeError("[NAN]")
    theta3_rad = np.arctan(np.sqrt(1-(cosq3)**2)/cosq3)
    theta2_rad = np.arctan(z/np.sqrt(x**2+y**2))-np.arctan((l3*np.sin(theta3_rad))/(l2+l3*np.cos(theta3_rad)))

    # Convertir a grados
    """ theta1_deg = np.degrees(theta1_rad)
    theta2_deg = np.degrees(theta2_rad)
    theta3_deg = np.degrees(theta3_rad) """
    
    #print(cosq3) # debug
    if theta2_rad > np.pi/2 or theta2_rad < -np.pi/2:
        raise Exception("[Fuera de rango]")
    if theta3_rad > np.pi/2 or theta3_rad < -np.pi/2:
        raise Exception("[Fuera de rango]")

    return theta1_rad, theta2_rad, theta3_rad

""" # Ejemplo de uso
x = 1  # posición deseada en x
y = 1  # posición deseada en y
z = 1

theta1, theta2, theta3 = inverse_kinematics(x, y,z)
print(f"Angulos de las articulaciones: theta1={theta1:.2f}, theta2={theta2:.2f}, theta3={theta3:.2f}") """

COM = 'COM5'
##DESCOMENTAR LA LINEA DE ABAJO PARA LA COMUNICACION SERIAL CON EL ARDUINO
#arduino = serial.Serial(COM,9600)
time.sleep(2)

while(True):
    msg = input("[Escriba el comando] => ")
    msg = msg.strip()
    msg = msg.split()
    for k in range(0,len(msg)):
        msg[k] = int(msg[k])
    #print(msg) #debug
    pr = msg[0:3]
    orient = msg[3:6]
    #Calculo del vector de orientacion del eje de la herramienta con respecto al eje de la base
    Transform = T(int(orient[0]), int(orient[1]), int(orient[2])) #Matriz de tranformacion directa
    pm = pr - np.cross(Transform[0:3,2],d) #Calculo de punto muñeca
    #print(pm) #debug
    #print(d)  #debug
    try:
        theta1_rad, theta2_rad, theta3_rad = inverse_kinematics(int(pm[0]), int(pm[1]), int(pm[2]))
        #Conversion de angulos a decimal
        theta1_deg = np.degrees(theta1_rad)
        theta2_deg = np.degrees(theta2_rad)
        theta3_deg = np.degrees(theta3_rad)
        A03 = multMatrizial(A01(theta1_rad),A12(theta2_rad))# los angulos aca deben estar en radianes
        A03 = multMatrizial(A03,A23(theta3_rad))
        #print(A03)#debug
        R03_inv = A03[0:3,0:3].transpose()
        #print(R03_inv)#debug
        R36 = R03_inv * Transform[0:3,0:3]
        #print(R36) #debug
        
        #Despejes para optener q4,q5,q6
        #cos(q5) = R36(2,2)
        #sen(q4) = R36(0,2)/sen(q5)
        #sen(q6) = R36(2,1)/sen(q5)   
        theta5_rad = np.arccos(R36[2,2]) 
        theta4_rad = np.arcsin(R36[0,2]/np.sin(theta5_rad))
        theta6_rad = np.arcsin(R36[2,1]/np.sin(theta5_rad))
        theta4_deg = np.degrees(theta4_rad)
        theta5_deg = np.degrees(theta5_rad)
        theta6_deg = np.degrees(theta6_rad)
        msg =  str(theta1_deg) + " , " + str(theta2_deg) + " , " + str(theta3_deg) + " , " + str(theta4_deg) + " , " + str(theta5_deg) + " , " + str(theta6_deg) + " , " + str(msg[-1])        
        print("[Mensaje enviado] => " + msg) 
        ## DESCOMENTAR LAS TRES LINEAS DE ABAJO PARA VERIFICAR LA RECEPCION DEL MENSAJE   
        #arduino.write(msg.encode())
        #respuesta = arduino.readline().decode().strip()
        #print("[Mensaje Recibido] => " + respuesta)
         
    except TypeError:
        print("[NAN] Combinacion de posicion no valida")
    except ZeroDivisionError:
        print("[Division entre cero]")
    except Exception:
        print("[Fuera de Rango]")
    