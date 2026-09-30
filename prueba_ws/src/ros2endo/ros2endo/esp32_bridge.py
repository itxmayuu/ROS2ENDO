#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import serial


class ESP32Bridge(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.puerto = '/dev/ttyUSB0'
        self.baudios = 921600
        self.omega_max = 3.1

        self.pub_omega1 = self.create_publisher(Float32, '/omega1', 10)
        self.pub_omega2 = self.create_publisher(Float32, '/omega2', 10)
        self.pub_imu = self.create_publisher(Float32, '/imu_heading', 10)

        self.sub_velocidad = self.create_subscription(
            String,
            '/velocidad_deseada',
            self.enviar_velocidad,
            10
        )

        try:
            self.serial_port = serial.Serial(
                self.puerto,
                self.baudios,
                timeout=0.05
            )
            self.serial_port.reset_input_buffer()
            self.get_logger().info(
                f'ESP32 conectado en {self.puerto} @ {self.baudios} baudios'
            )
        except serial.SerialException as e:
            self.get_logger().error(
                f'No se pudo abrir el puerto serial: {e}'
            )
            raise

        self.timer = self.create_timer(0.01, self.leer_serial)
        self.get_logger().info('Bridge listo: ROS 2 <-> ESP32')

    def enviar_velocidad(self, msg):
        try:
            partes = msg.data.strip().split(',')

            if len(partes) != 2:
                self.get_logger().warning(
                    f'Formato incorrecto: {msg.data}'
                )
                return

            w1 = float(partes[0])
            w2 = float(partes[1])

            if not (
                -self.omega_max <= w1 <= self.omega_max
                and -self.omega_max <= w2 <= self.omega_max
            ):
                self.get_logger().warning(
                    f'Referencia fuera de rango: w1={w1:.2f}, w2={w2:.2f}'
                )
                return

            comando = f'{w1:.3f},{w2:.3f}\n'
            self.serial_port.write(comando.encode('ascii'))

            self.get_logger().info(
                f'TX -> ESP32: w1={w1:.2f}, w2={w2:.2f}'
            )

        except ValueError:
            self.get_logger().warning(
                f'Valores no numericos: {msg.data}'
            )
        except serial.SerialException as e:
            self.get_logger().error(
                f'Error escribiendo al ESP32: {e}'
            )

    def leer_serial(self):
        try:
            raw = self.serial_port.read_until(b'\n')

            if not raw:
                return

            linea = raw.decode('ascii', errors='ignore').strip()

            if not linea:
                return

            if linea.startswith('>omega1:'):
                valor = float(linea.split(':', 1)[1])
                msg = Float32()
                msg.data = valor
                self.pub_omega1.publish(msg)

            elif linea.startswith('>omega2:'):
                valor = float(linea.split(':', 1)[1])
                msg = Float32()
                msg.data = valor
                self.pub_omega2.publish(msg)

            elif linea.startswith('>imu:'):
                valor = float(linea.split(':', 1)[1])
                msg = Float32()
                msg.data = valor
                self.pub_imu.publish(msg)

        except ValueError:
            self.get_logger().warning(
                f'Dato serial no valido: {linea}'
            )
        except serial.SerialException as e:
            self.get_logger().error(
                f'Error leyendo puerto serial: {e}'
            )

    def destroy_node(self):
        self.get_logger().info('Cerrando comunicacion con ESP32...')

        try:
            if hasattr(self, 'serial_port'):
                self.serial_port.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ESP32Bridge()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
