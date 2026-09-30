import os
import glob

import pandas as pd
import matplotlib.pyplot as plt


archivos = glob.glob('experimento_*.csv')

if not archivos:
    print('No se encontraron archivos experimento_*.csv')
    raise SystemExit

archivo = max(archivos, key=os.path.getmtime)
print(f'Analizando: {archivo}')

df = pd.read_csv(archivo)

t0 = df['ros_time_s'].iloc[0]
t = df['ros_time_s'] - t0

setpoint = 0.50
error = setpoint - df['lidar_left_m']

carpeta = 'plots'
os.makedirs(carpeta, exist_ok=True)

# 1. Distancia lateral
plt.figure(figsize=(10, 4))
plt.plot(t, df['lidar_left_m'], label='Distancia medida')
plt.axhline(setpoint, linestyle='--', label='Referencia = 0.50 m')
plt.xlabel('Tiempo [s]')
plt.ylabel('Distancia lateral [m]')
plt.title('Seguimiento lateral')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(f'{carpeta}/01_distancia_lateral.png', dpi=150)
plt.close()

# 2. Error lateral
plt.figure(figsize=(10, 4))
plt.plot(t, error)
plt.axhline(0.0, linestyle='--')
plt.xlabel('Tiempo [s]')
plt.ylabel('Error lateral [m]')
plt.title('Error lateral respecto a 0.50 m')
plt.grid(True)
plt.tight_layout()
plt.savefig(f'{carpeta}/02_error_lateral.png', dpi=150)
plt.close()

# 3. Heading IMU
plt.figure(figsize=(10, 4))
plt.plot(t, df['imu_heading_deg'])
plt.xlabel('Tiempo [s]')
plt.ylabel('Heading [grados]')
plt.title('Heading de la IMU')
plt.grid(True)
plt.tight_layout()
plt.savefig(f'{carpeta}/03_imu_heading.png', dpi=150)
plt.close()

# 4. Error de yaw
plt.figure(figsize=(10, 4))
plt.plot(t, df['imu_error_yaw_rad'])
plt.axhline(0.0, linestyle='--')
plt.xlabel('Tiempo [s]')
plt.ylabel('Error yaw [rad]')
plt.title('Error de orientacion')
plt.grid(True)
plt.tight_layout()
plt.savefig(f'{carpeta}/04_error_yaw.png', dpi=150)
plt.close()

# 5. Velocidades deseadas
plt.figure(figsize=(10, 4))
plt.plot(t, df['w_left_deseada_rad_s'], label='Izquierda deseada')
plt.plot(t, df['w_right_deseada_rad_s'], label='Derecha deseada')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad angular [rad/s]')
plt.title('Velocidades deseadas por el wall follower')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(f'{carpeta}/05_velocidades_deseadas.png', dpi=150)
plt.close()

# 6. Velocidades reales
plt.figure(figsize=(10, 4))
plt.plot(t, df['omega1_rad_s'], label='Omega 1')
plt.plot(t, df['omega2_rad_s'], label='Omega 2')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad angular [rad/s]')
plt.title('Velocidades reales de las ruedas')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(f'{carpeta}/06_velocidades_reales.png', dpi=150)
plt.close()

# 7. Rueda izquierda: deseada vs real
plt.figure(figsize=(10, 4))
plt.plot(t, df['w_left_deseada_rad_s'], label='Deseada')
plt.plot(t, df['omega1_rad_s'], label='Real')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad angular [rad/s]')
plt.title('Rueda izquierda: referencia vs respuesta')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(f'{carpeta}/07_rueda_izquierda.png', dpi=150)
plt.close()

# 8. Rueda derecha: deseada vs real
plt.figure(figsize=(10, 4))
plt.plot(t, df['w_right_deseada_rad_s'], label='Deseada')
plt.plot(t, df['omega2_rad_s'], label='Real')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad angular [rad/s]')
plt.title('Rueda derecha: referencia vs respuesta')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(f'{carpeta}/08_rueda_derecha.png', dpi=150)
plt.close()

error_valido = error.dropna()
mae = error_valido.abs().mean()
rmse = ((error_valido ** 2).mean()) ** 0.5
error_max = error_valido.abs().max()

print()
print('====================================')
print(' METRICAS DE SEGUIMIENTO LATERAL')
print('====================================')
print(f'MAE       = {mae:.4f} m')
print(f'RMSE      = {rmse:.4f} m')
print(f'Error max = {error_max:.4f} m')
print()
print(f'Graficas guardadas en: {carpeta}/')
