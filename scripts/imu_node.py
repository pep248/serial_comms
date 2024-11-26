#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial_communication.src.imu_class import MyIMU
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    imu_node = MyIMU()

    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(imu_node)

    try:
        # Run the nodes within the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()