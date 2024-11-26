#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import serial
import time

class MyGPS(Node):
    def __init__(self):
        super().__init__('gps_node')

        # Declare and get parameters
        port = self.declare_parameter('port', '/dev/ttyUSB1').get_parameter_value().string_value
        baudrate = self.declare_parameter('baudrate', 115200).get_parameter_value().integer_value

        # Initialize publisher and serial port
        self.publisher_ = self.create_publisher(String, 'gps_data', 10)
        try:
            self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            self.timer = self.create_timer(0.5, self.read_gps)  # 2 Hz timer
            self.get_logger().info(f"Initialized GPS Node on port {port} with baudrate {baudrate}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port {port}: {e}")
            self.serial_port = None

    def read_gps(self):
        if not self.serial_port:
            return

        try:
            if self.serial_port.in_waiting > 0:
                gps_data = self.serial_port.readline().decode('utf-8').strip()
                gps_msg = String(data=gps_data)
                self.publisher_.publish(gps_msg)
                self.get_logger().info(f"Published GPS data: {gps_data}")
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")