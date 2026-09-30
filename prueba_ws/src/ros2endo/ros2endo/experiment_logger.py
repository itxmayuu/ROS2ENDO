#!/usr/bin/env python3

import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class ExperimentLogger(Node):

    def __init__(self):
        super().__init__('experiment_logger')

        self.csv_dir = os.path.expanduser('~/ROS2ENDO_DATA')
        os.makedirs(self.csv_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.csv_dir,
            f'experimento_{timestamp}.csv'
        )

        self.csv_file = open(
            self.csv_path,
            'w',
            newline='',
            buffering=1
        )
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'ros_time_s',
            'lidar_left_m',
            'imu_heading_deg',
            'imu_error_yaw_rad',
            'w_left_deseada_rad_s',
            'w_right_deseada_rad_s',
            'omega1_rad_s',
            'omega2_rad_s'
        ])

        self.lidar_left = float('nan')
        self.imu_heading = float('nan')
        self.imu_error_yaw = float('nan')
        self.w_left_deseada = float('nan')
        self.w_right_deseada = float('nan')
        self.omega1 = float('nan')
        self.omega2 = float('nan')

        self.create_subscription(
            Float32,
            '/lidarLeft',
            self.callback_lidar,
            10
        )
        self.create_subscription(
            Float32,
            '/imu_heading',
            self.callback_imu,
            10
        )
        self.create_subscription(
            Float32,
            '/imu_error_yaw',
            self.callback_imu_error,
            10
        )
        self.create_subscription(
            String,
            '/velocidad_deseada',
            self.callback_velocidad,
            10
        )
        self.create_subscription(
            Float32,
            '/omega1',
            self.callback_omega1,
            10
        )
        self.create_subscription(
            Float32,
            '/omega2',
            self.callback_omega2,
            10
        )

        # Registro a 10 Hz.
        self.timer = self.create_timer(0.1, self.guardar_datos)

        self.get_logger().info('Experiment Logger iniciado')
        self.get_logger().info(f'CSV -> {self.csv_path}')

    def callback_lidar(self, msg):
        self.lidar_left = float(msg.data)

    def callback_imu(self, msg):
        self.imu_heading = float(msg.data)

    def callback_imu_error(self, msg):
        self.imu_error_yaw = float(msg.data)

    def callback_omega1(self, msg):
        self.omega1 = float(msg.data)

    def callback_omega2(self, msg):
        self.omega2 = float(msg.data)

    def callback_velocidad(self, msg):
        try:
            partes = msg.data.split(',')

            if len(partes) == 2:
                self.w_left_deseada = float(partes[0])
                self.w_right_deseada = float(partes[1])

        except ValueError:
            self.get_logger().warning(
                f'No se pudo interpretar /velocidad_deseada: {msg.data}'
            )

    def guardar_datos(self):
        tiempo = self.get_clock().now().nanoseconds / 1e9

        self.writer.writerow([
            f'{tiempo:.6f}',
            f'{self.lidar_left:.6f}',
            f'{self.imu_heading:.6f}',
            f'{self.imu_error_yaw:.6f}',
            f'{self.w_left_deseada:.6f}',
            f'{self.w_right_deseada:.6f}',
            f'{self.omega1:.6f}',
            f'{self.omega2:.6f}'
        ])

    def destroy_node(self):
        self.get_logger().info('Cerrando archivo CSV...')

        try:
            if hasattr(self, 'csv_file') and not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception as e:
            self.get_logger().warning(
                f'Error cerrando CSV: {e}'
            )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ExperimentLogger()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
