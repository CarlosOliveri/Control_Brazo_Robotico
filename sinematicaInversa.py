import numpy as np
import serial, time
l1 = 20.2
l2 = 16
l3 = 19.5

def inverse_kinematics(x, y,z):
    # Calcula theta3 usando la ley del coseno
    if x == 0:
        q1 = np.pi/2
    else:
        q1 = np.arctan(y/x)
    cosq3 = (x**2 + y**2 + z**2 - l2**2 - l3**2) / (2*l2*l3)
    if cosq3 > 1:
        raise TypeError("[NAN]")
    q3 = np.arctan(np.sqrt(1-(cosq3)**2)/cosq3)
    q2 = np.arctan(z/np.sqrt(x**2+y**2))-np.arctan((l3*np.sin(q3))/(l2+l3*np.cos(q3)))

    # Convertir a grados
    theta1_deg = np.degrees(q1)
    theta2_deg = np.degrees(q2)
    theta3_deg = np.degrees(q3)
    
    print(theta2_deg)
    if theta2_deg > 90 or theta2_deg < -90:
        raise Exception("[Fuera de rango]")
    if theta3_deg > 90 or theta3_deg < -90:
        raise Exception("[Fuera de rango]")

    return theta1_deg, theta2_deg, theta3_deg

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
    vec = msg.split()
    #print(vec) #debug
    try:
        theta1, theta2, theta3 = inverse_kinematics(int(vec[0]), int(vec[1]), int(vec[2]))
        print("[Mensaje enviado] => " + str(theta1) + " , " + str(theta2) + " , " + str(theta3)) 
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
         