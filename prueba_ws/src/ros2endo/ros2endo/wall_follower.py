#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')

        self.setpoint = 0.50
        self.omega_base = 1.60
        self.omega_min = 1.10
        self.omega_max = 2.10
        self.max_delta_omega = 0.12

        # Filtro pasa-bajos de distancia.
        self.alpha = 0.35
        self.distancia_filtrada = None

        # Variables para la derivada del error.
        self.error_anterior = None
        self.tiempo_anterior = None
        self.derivada_filtrada = 0.0
        self.beta_derivada = 0.30

        # Salidas anteriores para limitar cambios bruscos.
        self.w_left_anterior = self.omega_base
        self.w_right_anterior = self.omega_base

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

        self.get_logger().info('Wall follower FUZZY-PD iniciado')
        self.get_logger().info(
            f'Referencia lateral: {self.setpoint:.2f} m | '
            f'omega base: {self.omega_base:.2f} rad/s'
        )

    def crear_controlador_difuso(self):
        error = ctrl.Antecedent(
            np.arange(-1.50, 0.501, 0.001),
            'error'
        )

        derror = ctrl.Antecedent(
            np.arange(-0.80, 0.801, 0.001),
            'derror'
        )

        correccion = ctrl.Consequent(
            np.arange(-0.45, 0.451, 0.001),
            'correccion'
        )

        # Error lateral.
        error['NL'] = fuzz.trapmf(
            error.universe,
            [-1.50, -1.50, -0.30, -0.12]
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

        # Derivada del error.
        derror['N'] = fuzz.trapmf(
            derror.universe,
            [-0.80, -0.80, -0.18, -0.03]
        )
        derror['Z'] = fuzz.trimf(
            derror.universe,
            [-0.08, 0.0, 0.08]
        )
        derror['P'] = fuzz.trapmf(
            derror.universe,
            [0.03, 0.18, 0.80, 0.80]
        )

        # Correccion diferencial.
        correccion['NL'] = fuzz.trapmf(
            correccion.universe,
            [-0.45, -0.45, -0.38, -0.24]
        )
        correccion['NS'] = fuzz.trimf(
            correccion.universe,
            [-0.32, -0.18, 0.0]
        )
        correccion['Z'] = fuzz.trimf(
            correccion.universe,
            [-0.08, 0.0, 0.08]
        )
        correccion['PS'] = fuzz.trimf(
            correccion.universe,
            [0.0, 0.18, 0.32]
        )
        correccion['PL'] = fuzz.trapmf(
            correccion.universe,
            [0.24, 0.38, 0.45, 0.45]
        )

        reglas = [
            ctrl.Rule(error['NL'] & derror['N'], correccion['PL']),
            ctrl.Rule(error['NL'] & derror['Z'], correccion['PL']),
            ctrl.Rule(error['NL'] & derror['P'], correccion['PS']),

            ctrl.Rule(error['NS'] & derror['N'], correccion['PL']),
            ctrl.Rule(error['NS'] & derror['Z'], correccion['PS']),
            ctrl.Rule(error['NS'] & derror['P'], correccion['Z']),

            ctrl.Rule(error['Z'] & derror['N'], correccion['PS']),
            ctrl.Rule(error['Z'] & derror['Z'], correccion['Z']),
            ctrl.Rule(error['Z'] & derror['P'], correccion['NS']),

            ctrl.Rule(error['PS'] & derror['N'], correccion['Z']),
            ctrl.Rule(error['PS'] & derror['Z'], correccion['NS']),
            ctrl.Rule(error['PS'] & derror['P'], correccion['NL']),

            ctrl.Rule(error['PL'] & derror['N'], correccion['NS']),
            ctrl.Rule(error['PL'] & derror['Z'], correccion['NL']),
            ctrl.Rule(error['PL'] & derror['P'], correccion['NL']),
        ]

        sistema = ctrl.ControlSystem(reglas)
        self.simulador = ctrl.ControlSystemSimulation(sistema)

    def limitar_cambio(self, nuevo, anterior):
        cambio = nuevo - anterior

        if cambio > self.max_delta_omega:
            nuevo = anterior + self.max_delta_omega
        elif cambio < -self.max_delta_omega:
            nuevo = anterior - self.max_delta_omega

        return nuevo

    def lidar_callback(self, msg):
        distancia_cruda = float(msg.data)

        if self.distancia_filtrada is None:
            self.distancia_filtrada = distancia_cruda
        else:
            self.distancia_filtrada = (
                self.alpha * distancia_cruda
                + (1.0 - self.alpha) * self.distancia_filtrada
            )

        distancia = self.distancia_filtrada
        error = self.setpoint - distancia

        error_fuzzy = max(-1.49, min(0.49, error))

        ahora = self.get_clock().now()

        if self.error_anterior is None or self.tiempo_anterior is None:
            derror = 0.0
        else:
            dt = (ahora - self.tiempo_anterior).nanoseconds / 1e9

            if dt > 0.0:
                derror = (error - self.error_anterior) / dt
            else:
                derror = 0.0

        self.error_anterior = error
        self.tiempo_anterior = ahora

        self.derivada_filtrada = (
            self.beta_derivada * derror
            + (1.0 - self.beta_derivada) * self.derivada_filtrada
        )

        derror_fuzzy = max(
            -0.79,
            min(0.79, self.derivada_filtrada)
        )

        self.simulador.input['error'] = error_fuzzy
        self.simulador.input['derror'] = derror_fuzzy

        try:
            self.simulador.compute()
        except Exception as e:
            self.get_logger().warning(
                f'Error en fuzzy: {e}'
            )
            return

        u = self.simulador.output['correccion']

        w_left = self.omega_base + u
        w_right = self.omega_base - u

        w_left = max(self.omega_min, min(self.omega_max, w_left))
        w_right = max(self.omega_min, min(self.omega_max, w_right))

        w_left = self.limitar_cambio(
            w_left,
            self.w_left_anterior
        )
        w_right = self.limitar_cambio(
            w_right,
            self.w_right_anterior
        )

        self.w_left_anterior = w_left
        self.w_right_anterior = w_right

        mensaje = String()
        mensaje.data = f'{w_left:.2f},{w_right:.2f}'
        self.publisher_velocidad.publish(mensaje)

        self.get_logger().info(
            f'd={distancia_cruda:.3f} | '
            f'df={distancia:.3f} | '
            f'e={error:.3f} | '
            f'de={self.derivada_filtrada:.3f} | '
            f'u={u:.3f} | '
            f'wL={w_left:.2f} | '
            f'wR={w_right:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    nodo = WallFollower()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
