import numpy as np
import serial, time
l1 = 20.2 #cm
l2 = 16.0 #cm
l3 = 19.5 #cm
l4 = 6.715 #cm
d = np.array([l4, 0, 0])

A01 = lambda q1: np.array([ [np.cos(q1),    0, np.sin(q1),     0],
                            [np.sin(q1),    0, -np.cos(q1),    0],
                            [     0,        1,       0,       l1],
                            [     0,        0,       0,        1]])
A12 = lambda q2: np.array([ [np.cos(q2+np.pi/2), -np.sin(q2+np.pi/2),   0,  l2*np.cos(q2+np.pi/2)],
                            [np.sin(q2+np.pi/2), np.cos(q2+np.pi/2),    0,  l2*np.sin(q2+np.pi/2)],
                            [       0,                  0,              1,                      0],
                            [       0,                  0,              0,                      1]])
A23 = lambda q3: np.array([ [np.cos(q3-np.pi/2), 0, -np.sin(q3-np.pi/2),    0],
                            [np.sin(q3-np.pi/2), 0,  np.cos(q3-np.pi/2),    0],
                            [     0,            -1,       0,                0],
                            [     0,             0,       0,                1]])
A34 = lambda q4: np.array([ [np.cos(q4),    0,  np.sin(q4), 0],
                            [np.sin(q4),    0, -np.cos(q4), 0],
                            [     0,        1,       0,    l3],
                            [     0,        0,       0,     1]])
A45 = lambda q5: np.array([ [np.cos(q5+np.pi/2),    0,  np.sin(q5+np.pi/2), 0],
                            [np.sin(q5+np.pi/2),    0, -np.cos(q5+np.pi/2), 0],
                            [     0,                1,          0,          0],
                            [     0,                0,          0,          1]])
A56 = lambda q6: np.array([ [np.cos(q6), -np.sin(q6), 0,        0],
                            [np.sin(q6),  np.cos(q6), 0,        0],
                            [     0,         0,       1,       l4],
                            [     0,         0,       0,        1]])
""" T = lambda phi,theta,thi: np.array([[np.cos(phi)*np.cos(theta), np.cos(phi)*np.sin(theta)*np.sin(thi)-np.sin(phi)*np.sin(thi), np.cos(phi)*np.sin(theta)*np.cos(thi)+np.sin(phi)*np.sin(thi), 0],
                                    [np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta)*np.sin(thi)+np.cos(phi)*np.cos(thi), np.sin(phi)*np.sin(theta)*np.cos(thi)-np.cos(phi)*np.sin(thi), 0],
                                    [       -np.sin(theta),                 np.cos(theta)*np.sin(thi),                              np.cos(theta)*np.cos(thi),                                0],
                                    [             0,                                     0,                                                       0,                                           1]])"""
#phi = z = alfa
# theta = y = beta
# thi = x = gama

def direct_kinematics(q1,q2,q3,q4,q5,q6):
    """ q1 = q1*np.pi/180
    q2 = q2*np.pi/180
    q3 = q3*np.pi/180
    q4 = q4*np.pi/180
    q5 = q5*np.pi/180
    q6 = q6*np.pi/180 """
    _0A2 = np.dot(A01(q1),A12(q2))
    _0A3 = np.dot(_0A2,A23(q3))
    _0A4 = np.dot(_0A3,A34(q4))
    _0A5 = np.dot(_0A4,A45(q5))
    _0A6 = np.dot(_0A5,A56(q6))
    return _0A6     

def inverse_kinematics(x, y, z):
    # Calcula theta3 usando la ley del coseno
    if x == 0:
        theta1_rad = np.pi/2
    else:
        theta1_rad = np.arctan(y/x)
    cosq3 = (x**2 + y**2 + (z-l1)**2 - l2**2 - l3**2) / (2*l2*l3)
    print("cosq3:" + str(cosq3))
    if abs(cosq3) > 1:
        cosq3 = 1
    theta3_rad = np.arctan2(np.sqrt(1-(cosq3)**2),cosq3)
    u = np.sqrt(x**2 + y**2 + (z-l1)**2)
    theta2_rad =  np.pi/2 - (np.arctan2((z-l1),np.sqrt(x**2 + y**2)) + np.arccos((l2**2 + u**2 - l3**2)/(2*l2*u)))

    # Convertir a grados
    """ theta1_deg = np.degrees(theta1_rad)
    theta2_deg = np.degrees(theta2_rad)
    theta3_deg = np.degrees(theta3_rad) """
    print(theta2_rad,theta3_rad)
    #print(theta2_rad) # debug
    if abs(theta2_rad) > np.pi/2:
        raise Exception("[Fuera de rango]")
    if abs(theta3_rad) > np.pi/2:
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
    #Comvierto a enteros el vector introducido
    for k in range(0,len(msg)):
        msg[k] = float(msg[k])*np.pi/180
    #print(msg) #debug
    
    print("Cinematica Directa:")
    Transform = direct_kinematics(msg[0],msg[1],msg[2],msg[3],msg[4],msg[5])
    print(Transform)
    
    pm = Transform[0:3,3] - Transform[0:3,2]*l4
    print("Punto muñeca: ")
    print(pm)
    theta1_rad, theta2_rad, theta3_rad = inverse_kinematics(float(pm[0]), float(pm[1]), float(pm[2]))
    #Conversion de angulos a decimal
    theta1_deg = np.degrees(theta1_rad)
    theta2_deg = np.degrees(theta2_rad)
    theta3_deg = np.degrees(theta3_rad)
    
    
    msg =  str(theta1_deg) + " , " + str(theta2_deg) + " , " + str(theta3_deg) 
    print("[Mensaje enviado] => " + msg) 
    