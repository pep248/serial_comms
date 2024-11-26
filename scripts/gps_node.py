#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial_communication.src.gps_class import MyGPS
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    gps_node = MyGPS()

    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(gps_node)

    try:
        # Run the nodes within the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()