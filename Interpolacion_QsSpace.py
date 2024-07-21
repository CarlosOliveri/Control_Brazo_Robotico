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

""" points = [[0, 0, 0],        # Punto 1
    [15, 15, 15],  # Punto 2
    [30, 30, 30],  # Punto 4
    [60, 60, 60],  # Punto 3
    [90, 85, 90],   # Punto 5 ##Leer
    [110, 60, 90]]
points = np.array(points) """
points = np.array([
    [0, 0, 0],        # Punto 1
    [15, 15, 15],  # Punto 2
    [30, 30, 30],  # Punto 4
    [60, 60, 60],  # Punto 3
    [90, 85, 90],   # Punto 5 ##Leer
    [110, 60, 90] ])

# Extraer los puntos individuales
q1_points = points[:, 0]
q2_points = points[:, 1]
q3_points = points[:, 2]

# Crear una secuencia de valores t para los puntos originales
t_points = np.linspace(0, 1, len(points))

# Crear una secuencia de valores t para la interpolación (30 puntos)
t_interpolated = np.linspace(0, 1, 30)

# Realizar la interpolación cúbica
cs_x = CubicSpline(t_points, q1_points,bc_type = 'clamped')
cs_y = CubicSpline(t_points, q2_points,bc_type = 'clamped')
cs_z = CubicSpline(t_points, q3_points,bc_type = 'clamped')

# Generar los puntos interpolados
q1_interpolated = cs_x(t_interpolated)
q2_interpolated = cs_y(t_interpolated)
q3_interpolated = cs_z(t_interpolated)

# Guardar los puntos interpolados en una matriz
interpolated_points = np.vstack((q1_interpolated, q2_interpolated, q3_interpolated)).T

print("Puntos de la interpolacion:")
print(interpolated_points)

# Visualizar la trayectoria original e interpolada
fig = plt.figure()
ax = fig.add_subplot(311)
ax.plot( t_interpolated,q1_interpolated, '-', label='Puntos Interpolados y corregidos')
ax.plot( t_points,q1_points, '-', label='Puntos originales')
ax = fig.add_subplot(312)
ax.plot( t_interpolated,q2_interpolated, '-', label='Puntos Interpoladis y corregidos')
ax.plot( t_points,q2_points, '-', label='Puntos originales')
ax = fig.add_subplot(313)
ax.plot( t_interpolated,q3_interpolated, '-', label='Puntos Interpolados y corregidos')
ax.plot( t_points,q3_points, '-', label='Puntos originales')
ax.legend()
plt.show()