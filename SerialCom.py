#import roboticstoolbox as rtb
import serial, time
COM = 'COM5'
arduino = serial.Serial(COM,9600)
time.sleep(2)
while(True):
    msg = input("escriba el comando: ")
    vec = msg.split(',')
    arduino.write(msg.encode())
    respuesta = arduino.readline().decode().strip()
    print(respuesta)
    
    