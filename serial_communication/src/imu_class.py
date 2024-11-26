#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import serial
import time

class MyIMU(Node):
    def __init__(self):
        super().__init__('my_imu_node')

        # Declare and get parameters
        port = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        baudrate = self.declare_parameter('baudrate', 115200).get_parameter_value().integer_value

        # Initialize publisher and serial port
        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu_data', 10)
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            self.timer = self.create_timer(0.1, self.read_imu)  # 10 Hz timer
            self.get_logger().info(f"Initialized IMU Node on port {port} with baudrate {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port {port}: {e}")
            self.serial_port = None

    def read_imu(self):
        if not self.serial_port:
            return
        else:
            try:
                if self.serial_port.in_waiting > 0:
                    imu_data = self.serial_port.readline().decode('utf-8').strip()
                    # Assuming IMU data is comma-separated values: ax, ay, az, gx, gy, gz, mx, my, mz
                    values = [float(v) for v in imu_data.split(',')]
                    imu_msg = Float32MultiArray(data=values)
                    self.publisher_.publish(imu_msg)
                    self.get_logger().info(f"Published IMU data: {values}")
            except Exception as e:
                self.get_logger().error(f"Error reading IMU data: {e}")