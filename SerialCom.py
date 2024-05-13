import serial, time
COM = input("Ingrese el puerto COM5M#:")
arduino = serial.Serial(COM,9600)
time.sleep(2)
while(True):
    msg = input("escriba el comando: ")
    arduino.write(msg.encode())
    respuesta = arduino.readline().decode().strip()
    print(respuesta)
    
    