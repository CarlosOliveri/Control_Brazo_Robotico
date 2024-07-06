import numpy as np
l1 = 20.2
l2 = 16
l3 = 19.5
def inverse_kinematics(x, y,z, l1, l2, l3):
    # Calcula theta3 usando la ley del coseno
    q1 = np.arctan(y/x)
    cosq3 = (x*x+y*y+z*z-l2*l2-l3*l3)/2*l2*l3
    q3 = np.arctan(np.sqrt(1-(cosq3)**2)/cosq3)
    q2 = np.arctan(z/np.sqrt(x**2+y**2))-np.arctan((l3*np.sin(q3))/(l2+l3*np.cos(q3)))

    # Convertir a grados
    theta1_deg = np.degrees(q1)
    theta2_deg = np.degrees(q2)
    theta3_deg = np.degrees(q3)

    return theta1_deg, theta2_deg, theta3_deg

# Ejemplo de uso
x = 1  # posición deseada en x
y = 1  # posición deseada en y
z = 1

theta1, theta2, theta3 = inverse_kinematics(x, y,z,l1, l2, l3)
print(f"Angulos de las articulaciones: theta1={theta1:.2f}, theta2={theta2:.2f}, theta3={theta3:.2f}")

import serial, time
COM = 'COM5'
#arduino = serial.Serial(COM,9600)
time.sleep(2)

while(True):
    msg = input("escriba el comando: ")
    vec = msg.split(',')
    print(vec)
    theta1, theta2, theta3 = inverse_kinematics((vec[0]), (vec[1]),(vec[2]),l1, l2, l3)
        
    arduino.write(msg.encode())
    respuesta = arduino.readline().decode().strip()
    print(respuesta) """
    