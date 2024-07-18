import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# Definir los 5 puntos (x, y, z) a través de los cuales debe pasar la trayectoria
points = np.array([
    [15, 15,  35],        # Punto 1
    [21.7,  13.2,  37.7],  # Punto 2
    [23.9, 12.9, 43.5],  # Punto 4
    [ 10.2, 14.5, 44.2],  # Punto 3
    [-8,  16.5,  45.9],   # Punto 5 ##Leer
    [-12.4,  15.7,  45.7]
]) # STR 1 0 2 0.1 0.2 0.3 0.2 0.1 0.4 0.3 0.4 0.5 0.4 0.3 0.6

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
ax.plot(x_points, y_points, z_points, 'o', label='Puntos originales')
ax.plot(x_interpolated, y_interpolated, z_interpolated, '-', label='Trayectoria interpolada')

ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Trayectoria Interpolada del Brazo Robótico Thor')
ax.legend()
plt.show()