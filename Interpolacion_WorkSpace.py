import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Definir los 5 puntos (x, y, z) a través de los cuales debe pasar la trayectoria

""" points = np.array([
    [-10, 20, 21],        # Punto 1
    [-20, 20, 30],  # Punto 2
    [-20, 20, 40],  # Punto 4
    [ -20, 10, 30],  # Punto 3
    [-10, 20, 30],   # Punto 5 ##Leer
    [-10, 20, 30] #STR -10 20 21 -20 20 30 -20 20 35 -20 10 30 -10 20 40.2 -10 20 30.2
]) """

""" points = np.array([
    [15, 25,  28.2],        # Punto 1
    [0,  25,  43.2],  # Punto 2
    [-15, 25, 28.2],  # Punto 4
    [ 0, 25, 13.2],  # Punto 3
    [15,  25,  28.2],   # Punto 5 ##Leer
    [0,  25,  43.2]
]) """ # STR 1 0 2 0.1 0.2 0.3 0.2 0.1 0.4 0.3 0.4 0.5 0.4 0.3 0.6  una s en el espacio

points = np.array([
    [15, 15,  35],        # Punto 1
    [21.7,  13.2,  37.7],  # Punto 2
    [23.9, 12.9, 43.5],  # Punto 4
    [ 10.2, 14.5, 44.2],  # Punto 3
    [-8,  16.5,  45.9],   # Punto 5 ##Leer
    [-12.4,  15.7,  45.7] ])
    
    
def cartesian2polar(x,y,z):
    r = np.sqrt(x**2 + y**2 + z**2)
    Q = np.degrees(np.arctan2(y,x)).round(decimals=3)
    phi = np.degrees(np.arccos(z/r)).round(decimals=3)
    return r,Q,phi
def polar2cartesian(r,q,phi):
    q = q*np.pi/180
    phi = phi*np.pi/180
    x = r*np.sin(phi)*np.cos(q)
    y = r*np.sin(phi)*np.sin(q)
    z = r*np.cos(phi)
    return x, y, z

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

print("Puntos de la interpolacion:")
print(interpolated_points)

# Visualizar la trayectoria original e interpolada
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_points, y_points, z_points, '*', label='Puntos originales')
ax.plot(x_interpolated, y_interpolated, z_interpolated, 'o', label='Puntos Interpolados')
ax.plot(x_interpolated, y_interpolated, z_interpolated, '-', label='Trayectoria interpolada')

x_corregido = []
y_corregido = []
z_corregido = []
for k in interpolated_points:
    k[2] = k[2] - 20.2
    r,q,phi = cartesian2polar(k[0], k[1], k[2]) #Max = 40.415 Min = 25.224 
    print(r,q,phi)
    if phi < -49:
        phi = -48
    if r < 25.2:
        r = 25.2
        x, y, z = polar2cartesian(r,q,phi)
        x_corregido.append(x)
        y_corregido.append(y)
        z_corregido.append(z + 20.2)
        #print("new:")
        #print(r,q,phi)
    elif r > 35.4:
        r = 35.4
        x, y, z = polar2cartesian(r,q,phi)
        x_corregido.append(x)
        y_corregido.append(y)
        z_corregido.append(z + 20.2)
        #print("new:")
    else:
        x, y, z = polar2cartesian(r,q,phi)
        x_corregido.append(x)
        y_corregido.append(y)
        z_corregido.append(z + 20.2)
    #print(r,q,phi) 
    #print("------------")
interpolated_points = np.vstack((x_corregido, y_corregido, z_corregido)).T
print("____________________________________")
print(interpolated_points)
    
ax.plot(x_corregido, y_corregido, z_corregido, '-', label='Trayectoria corregida')

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Trayectoria Interpolada del Brazo Robótico Thor')
ax.legend()
plt.show()
plt.show()