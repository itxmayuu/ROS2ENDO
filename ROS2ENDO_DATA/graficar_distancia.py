import pandas as pd
import matplotlib.pyplot as plt

archivo = 'experimento_20260829_214651.csv'

df = pd.read_csv(archivo)

t0 = df['ros_time_s'].iloc[0]
tiempo = df['ros_time_s'] - t0

plt.figure(figsize=(10, 4))

plt.plot(
    tiempo,
    df['lidar_left_m'],
    label='Distancia medida'
)

plt.axhline(
    0.50,
    linestyle='--',
    label='Referencia = 0.50 m'
)

plt.xlabel('Tiempo [s]')
plt.ylabel('Distancia lateral [m]')
plt.title('Seguimiento lateral del robot')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig('distancia_lateral.png', dpi=150)

print('Grafica guardada como distancia_lateral.png')
