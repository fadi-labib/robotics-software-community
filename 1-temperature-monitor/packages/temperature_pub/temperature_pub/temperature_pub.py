#!/usr/bin/env python3
import rclpy
# import math
# import numpy as np
import random
from rclpy.node import Node

from temperature_interfaces.msg import Temperature


class TemperaturePublisherNode(Node):
    def __init__(self):
        super().__init__("temperature_pub")


        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("sensor_id", "sensor_1")
        self.declare_parameter("min_temp", 10)
        self.declare_parameter("max_temp", 50)

    
        self.publish_frequency_ = self.get_parameter("publish_frequency").value
        self.sensor_id_ = self.get_parameter("sensor_id").value
        self.min_temp_ = self.get_parameter("min_temp").value
        self.max_temp_ = self.get_parameter("max_temp").value

        self.temp_publisher_ = self.create_publisher(Temperature, "temperature", 10)

        self.timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_temperature)

        self.get_logger().info("Temperature publisher has been started.")

    def publish_temperature(self):
        msg = Temperature()

        # msg.value = randomize_temperature()

        msg.value = (self.min_temp_ + self.max_temp_)/2 +random.uniform(-5,5)
        msg.time_stamp = self.get_clock().now().to_msg()
        msg.sensor_id = self.sensor_id_

        msg.is_celsius = True 

        self.temp_publisher_.publish(msg)
        self.get_logger().info(
        f'Published: {msg.value:.2f}°{"C" if msg.is_celsius else "F"} from {msg.sensor_id}'
        )


    # def randomize_temperature(self):

        #To be implemented

        # return temperature


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()