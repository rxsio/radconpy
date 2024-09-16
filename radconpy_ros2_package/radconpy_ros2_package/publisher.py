#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Int64
from radcon_msgs.msg import RadconMeasurement
import random
import time

from radcompy_ros2_package.radconpy.collector import RadConCollector
from radcompy_ros2_package.radconpy.device import RadConDevice
from radcompy_ros2_package.radconpy.manager import Measurement, RadConManager


class RadcomPublisher(Node):

    def __init__(self, radcon_manager):
        super().__init__('radcon publisher')
        self.publisher_ = self.create_publisher(FluidPressure, 'radcon_measurement', 10)
        self.radcon_manager = radcon_manager
        timer_period = 0.5  #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        measurement= self.radcon_manager.get_measurement()

        msg = RadconMeasurement()
        msg.header.stamp = measurement.timestamp

        msg.hardware_timestamp.sec = measurement.hardware_timestamp // 1e3 
        msg.hardware_timestamp.nanosec = (measurement.hardware_timestamp % 1e3) * 1e6  

        msg.pulse_length.sec = measurement.pulse_length // 1e6
        msg.pulse_length.nanosec = (measurement.pulse_length % 1e6) * 1e3

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class MockRadConManager(RadConManager):

    def get_measurement(self, max_tries: int = 3) -> Measurement | None:
        time.sleep(random.randint(100, 300) / 100)
        return 3
    
    def get_firmware(self, max_tries: int = 3) -> str | None:
        return "MOCK"

def main(args=None):

    r = RadConDevice("COM3", logger_level="DEBUG")
    m = MockRadConManager(r, logger_level="DEBUG", reconnect_cooldown=2.0)
    c = RadConCollector(m, "m1.csv", timebase=60)
    print(f"Firmware: {m.get_firmware(3)}")
    c.run(visualize=False)

    rclpy.init(args=args)
    minimal_publisher = RadcomPublisher(m)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
