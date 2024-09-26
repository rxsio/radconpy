#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from radcon_msgs.msg import RadconMeasurement
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from radcon.radcon import RadCon
#import sys
#sys.path.append('/workspaces/firo/src/radconpy/radconpy_ros2_package/radconpy/radconpy')
#from radcon import RadCon


class RadconPublisher(Node):

    def __init__(self):
        super().__init__('RadconPublisher')

        self.declare_parameter('port', 'COM3') 
        self.declare_parameter('reconnect_cooldown', 1.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        reconnect_cooldown = self.get_parameter('reconnect_cooldown').get_parameter_value().double_value
        
        self.radcon_device = RadCon(port, reconnect_cooldown=reconnect_cooldown)
        self.publisher_ = self.create_publisher(RadconMeasurement, 'radcon_measurement', 10)

        self.radcon_device.on_connected.add(self.on_con)
        self.radcon_device.on_disconnected.add(self.on_dis)
        self.radcon_device.on_data.add(self.on_data)
        
        self.radcon_device.start()


    def publish_measurement(self, hardware_timestamp, pulse_length):

        msg = RadconMeasurement()
        msg.header.stamp = Clock().now().to_msg()

        msg.hardware_timestamp.sec = int(hardware_timestamp // 1e3)
        msg.hardware_timestamp.nanosec = int((hardware_timestamp % 1e3) * 1e6)

        msg.pulse_length.sec = int(pulse_length // 1e6)
        msg.pulse_length.nanosec = int((pulse_length % 1e6) * 1e3)

        self.publisher_.publish(msg)


    def on_con(self):
        self.get_logger().info('Connected')

    def on_dis(self):
        self.get_logger().info('Disconnected')

    def on_data(self, timestamp, hardware_timestamp, pulse_length):
        self.publish_measurement(hardware_timestamp, pulse_length)
    
def main(args=None):


    rclpy.init(args=args)
    try:
        radcon_publisher = RadconPublisher()
        rclpy.spin(radcon_publisher)
    
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
        radcon_publisher.destroy_node()


if __name__ == '__main__':

    main()
