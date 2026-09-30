#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import ydlidar


def wrap_to_pi(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        self.puerto = (
            '/dev/serial/by-id/'
            'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )

        # Cono total actual: 40 grados = +/-20 grados.
        self.apertura_total_deg = 40.0
        self.apertura_media = math.radians(self.apertura_total_deg / 2.0)

        # Direccion lateral izquierda del robot.
        self.centro_base = -math.pi / 2.0

        self.max_range = 8.0
        self.max_proj = 2.0

        self.imu_inicial = None
        self.imu_actual = 0.0
        self.e_yaw = 0.0

        self.pub_lidar_left = self.create_publisher(
            Float32,
            '/lidarLeft',
            10
        )

        self.pub_error_yaw = self.create_publisher(
            Float32,
            '/imu_error_yaw',
            10
        )

        self.sub_imu = self.create_subscription(
            Float32,
            '/imu_heading',
            self.callback_imu,
            10
        )

        self.get_logger().info('Iniciando YDLidar...')

        ydlidar.os_init()
        self.laser = ydlidar.CYdLidar()

        self.laser.setlidaropt(
            ydlidar.LidarPropSerialPort,
            self.puerto
        )
        self.laser.setlidaropt(
            ydlidar.LidarPropSerialBaudrate,
            115200
        )
        self.laser.setlidaropt(
            ydlidar.LidarPropScanFrequency,
            10.0
        )
        self.laser.setlidaropt(
            ydlidar.LidarPropSingleChannel,
            True
        )

        if not self.laser.initialize():
            raise RuntimeError('No fue posible inicializar el YDLidar')

        if not self.laser.turnOn():
            raise RuntimeError('No fue posible encender el YDLidar')

        self.get_logger().info(
            f'YDLidar conectado en {self.puerto}'
        )
        self.get_logger().info(
            'LiDAR configurado a 115200 baudios y 10 Hz'
        )
        self.get_logger().info(
            f'Cono lateral configurado a {self.apertura_total_deg:.1f} '
            f'grados totales (+/-{self.apertura_total_deg/2.0:.1f} grados)'
        )

        self.timer = self.create_timer(0.1, self.leer_lidar)

    def callback_imu(self, msg):
        angulo_actual = math.radians(float(msg.data))
        angulo_actual = wrap_to_pi(angulo_actual)

        if self.imu_inicial is None:
            self.imu_inicial = angulo_actual
            self.get_logger().info(
                f'Referencia IMU inicial: '
                f'{math.degrees(self.imu_inicial):.2f} grados'
            )

        self.imu_actual = angulo_actual
        self.e_yaw = wrap_to_pi(
            self.imu_inicial - self.imu_actual
        )

        msg_error = Float32()
        msg_error.data = float(self.e_yaw)
        self.pub_error_yaw.publish(msg_error)

    def leer_lidar(self):
        scan = ydlidar.LaserScan()
        ok = self.laser.doProcessSimple(scan)

        if not ok or len(scan.points) == 0:
            self.get_logger().warning(
                'No se pudo obtener el barrido del LiDAR'
            )
            return

        centro_cono = wrap_to_pi(
            self.centro_base + self.e_yaw
        )

        suma = 0.0
        contador = 0

        for punto in scan.points:
            angulo = wrap_to_pi(punto.angle)
            diferencia = wrap_to_pi(angulo - centro_cono)

            if abs(diferencia) <= self.apertura_media:
                distancia = punto.range

                if 0.0 < distancia <= self.max_range:
                    proyeccion = distancia * math.cos(diferencia)

                    if 0.0 < proyeccion <= self.max_proj:
                        suma += proyeccion
                        contador += 1

        if contador > 0:
            distancia_lateral = suma / contador

            msg = Float32()
            msg.data = float(distancia_lateral)
            self.pub_lidar_left.publish(msg)

            self.get_logger().info(
                f'Distancia lateral: {distancia_lateral:.3f} m | '
                f'validas: {contador} | '
                f'eYaw: {math.degrees(self.e_yaw):.2f} grados | '
                f'centro cono: {math.degrees(centro_cono):.2f} grados'
            )
        else:
            self.get_logger().warning(
                'No hay puntos validos dentro del cono de 40 grados'
            )

    def destroy_node(self):
        self.get_logger().info('Apagando YDLidar...')

        try:
            self.laser.turnOff()
            self.laser.disconnecting()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = LidarNode()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
