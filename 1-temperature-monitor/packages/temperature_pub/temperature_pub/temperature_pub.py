#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node

from temperature_interfaces.msg import Temperature


class TemperaturePublisherNode(Node):
    def __init__(self):
        super().__init__("temperature_pub")

        # Parameter declaration and default values
        self.declare_parameter("publish_frequency", 1.0)
        self.declare_parameter("sensor_id", "sensor_1")
        self.declare_parameter("min_temp", 10)
        self.declare_parameter("max_temp", 50)

        # Assigning parameters to internal variables
        self.publish_frequency_ = self.get_parameter("publish_frequency").value
        self.sensor_id_ = self.get_parameter("sensor_id").value
        self.min_temp_ = self.get_parameter("min_temp").value
        self.max_temp_ = self.get_parameter("max_temp").value

        self.temp_publisher_ = self.create_publisher(Temperature, "temperature", 10)

        self.timer_ = self.create_timer(1.0 / self.publish_frequency_, self.publish_temperature)

        self.get_logger().info("Temperature publisher has been started.")

    def publish_temperature(self):
        msg = Temperature()

        msg.value = self.randomize_temperature()

        msg.time_stamp = self.get_clock().now().to_msg()
        msg.sensor_id = self.sensor_id_

        # For clarity only as it's true by default
        msg.is_celsius = True 

        self.temp_publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.value:.2f}°{"C" if msg.is_celsius else "F"} from {msg.sensor_id}')


    def randomize_temperature(self):
        # Returns temperature values 
        # values range from [min_temp,max_temp] + gaussian noise


        # Generate a phase for the sine wave
        # Used clock for generation to simulate that values depend on time of "measurement" 
        seconds=self.get_clock().now().nanoseconds / 1e9 

       
        cycle_period_sec =200
        sin_phase_fraction= (seconds % cycle_period_sec)/cycle_period_sec

        # Generate a sine wave (it outputs [-1,1])
        sin_value=np.sin(2*np.pi*sin_phase_fraction)

        # Map [-1,1] to [min_temp,max_temp]
        temperature=(sin_value+1)/2 * (self.max_temp_-self.min_temp_) + self.min_temp_

        temperature_noisy=temperature + np.random.normal(0,1)

        return temperature_noisy


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()