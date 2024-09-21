#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from radcon_msgs.msg import RadconMeasurement
import random
import time
from radconpy_ros2_package.radconpy.radcon import RadCon


class RadconPublisher(Node):

    def __init__(self):
        super().__init__('RadconPublisher')

        self.declare_parameter('port', 'COM3') 
        self.declare_parameter('reconnect_cooldown', 1.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        reconnect_cooldown = self.get_parameter('reconnect_cooldown').get_parameter_value().double_value
        
        self.radcon_device = RadCon(port, reconnect_cooldown=reconnect_cooldown)

        
        self.publisher_ = self.create_publisher(RadconMeasurement, 'radcon_measurement', 10)
        timer_period = 0.5  #seconds
        self.timer = self.create_timer(timer_period, self.publish_measurement)
        self.i = 0

        self.radcon_device.on_connected.add(self.on_con)
        self.radcon_device.on_disconnected.add(self.on_dis)
        self.radcon_device.on_data.add(self.on_data)
        
        self.radcon_device.start()


    def publish_measurement(self, timestamp, hardware_timestamp, pulse_length):

        msg = RadconMeasurement()
        msg.header.stamp = timestamp

        msg.hardware_timestamp.sec = hardware_timestamp // 1e3 
        msg.hardware_timestamp.nanosec = (hardware_timestamp % 1e3) * 1e6  

        msg.pulse_length.sec = pulse_length // 1e6
        msg.pulse_length.nanosec = (pulse_length % 1e6) * 1e3

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def on_con(self):
        print("Connected")

    def on_dis(self):
        print("Disconnected")

    def on_data(self,timestamp, hardware_timestamp, pulse_length):
        self.radcon_publisher.publish_measurement(timestamp, hardware_timestamp, pulse_length)
    
def main(args=None):


    rclpy.init(args=args)
    radcon_publisher = RadconPublisher()
    rclpy.spin(radcon_publisher)
    radcon_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
