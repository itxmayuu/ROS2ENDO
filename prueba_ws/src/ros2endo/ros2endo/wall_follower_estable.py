#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class WallFollowerEstable(Node):

    def __init__(self):
        super().__init__('wall_follower_estable')

        self.setpoint = 0.50

        self.subscription = self.create_subscription(
            Float32,
            '/lidarLeft',
            self.lidar_callback,
            10
        )

        self.publisher_velocidad = self.create_publisher(
            String,
            '/velocidad_deseada',
            10
        )

        self.crear_controlador_difuso()

        self.get_logger().info('Nodo wall_follower_estable iniciado')
        self.get_logger().info(
            f'Distancia lateral deseada: {self.setpoint:.2f} m'
        )

    def crear_controlador_difuso(self):
        error = ctrl.Antecedent(
            np.arange(-1.5, 0.501, 0.001),
            'error'
        )

        w_left = ctrl.Consequent(
            np.arange(0.5, 3.001, 0.001),
            'w_left'
        )
        w_right = ctrl.Consequent(
            np.arange(0.5, 3.001, 0.001),
            'w_right'
        )

        # Membresias del error lateral.
        error['NL'] = fuzz.trapmf(
            error.universe,
            [-1.5, -1.5, -0.30, -0.12]
        )
        error['NS'] = fuzz.trimf(
            error.universe,
            [-0.25, -0.10, 0.0]
        )
        error['Z'] = fuzz.trimf(
            error.universe,
            [-0.06, 0.0, 0.06]
        )
        error['PS'] = fuzz.trimf(
            error.universe,
            [0.0, 0.10, 0.25]
        )
        error['PL'] = fuzz.trapmf(
            error.universe,
            [0.12, 0.30, 0.50, 0.50]
        )

        # Salidas suavizadas que funcionaron bien experimentalmente.
        for salida in (w_left, w_right):
            salida['slow'] = fuzz.trimf(
                salida.universe,
                [1.0, 1.2, 1.4]
            )
            salida['medium'] = fuzz.trimf(
                salida.universe,
                [1.3, 1.6, 1.9]
            )
            salida['fast'] = fuzz.trimf(
                salida.universe,
                [1.8, 2.0, 2.2]
            )

        reglas = [
            ctrl.Rule(
                error['NL'],
                consequent=[w_left['fast'], w_right['slow']]
            ),
            ctrl.Rule(
                error['NS'],
                consequent=[w_left['medium'], w_right['slow']]
            ),
            ctrl.Rule(
                error['Z'],
                consequent=[w_left['medium'], w_right['medium']]
            ),
            ctrl.Rule(
                error['PS'],
                consequent=[w_left['slow'], w_right['medium']]
            ),
            ctrl.Rule(
                error['PL'],
                consequent=[w_left['slow'], w_right['fast']]
            ),
        ]

        sistema = ctrl.ControlSystem(reglas)
        self.simulador = ctrl.ControlSystemSimulation(sistema)
        self.error_var = error

    def lidar_callback(self, msg):
        distancia = float(msg.data)
        error_distancia = self.setpoint - distancia

        # Saturacion para evitar salir del universo definido.
        error_fuzzy = max(-1.49, min(0.49, error_distancia))
        self.simulador.input['error'] = error_fuzzy

        try:
            self.simulador.compute()
        except Exception as e:
            self.get_logger().warning(
                f'No fue posible calcular el fuzzy: {e}'
            )
            return

        w_left = self.simulador.output['w_left']
        w_right = self.simulador.output['w_right']

        membresias = {}
        for etiqueta in ['NL', 'NS', 'Z', 'PS', 'PL']:
            membresias[etiqueta] = fuzz.interp_membership(
                self.error_var.universe,
                self.error_var[etiqueta].mf,
                error_fuzzy
            )

        texto_membresias = ', '.join(
            f'{nombre}:{valor:.2f}'
            for nombre, valor in membresias.items()
        )

        self.get_logger().info(
            f'Distancia: {distancia:.3f} m | '
            f'Error: {error_distancia:.3f} m | '
            f'{texto_membresias} | '
            f'wL={w_left:.2f} rad/s | '
            f'wR={w_right:.2f} rad/s'
        )

        mensaje = String()
        mensaje.data = f'{w_left:.2f},{w_right:.2f}'
        self.publisher_velocidad.publish(mensaje)


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollowerEstable()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
