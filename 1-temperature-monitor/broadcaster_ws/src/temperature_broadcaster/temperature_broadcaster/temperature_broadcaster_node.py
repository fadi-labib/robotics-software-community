# temperature_publisher_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from custom_temperature_msgs.msg import TemperatureReading
from temperature_broadcaster.temperature_simulator import TemperatureSimulator
import argparse


class TemperaturePublisher(Node):
    def __init__(self, freq_hz, min_temp, max_temp, sensor_id):
        super().__init__('temperature_publisher')

        self.sensor_id = sensor_id
        self.simulator = TemperatureSimulator(min_temp, max_temp)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(
            TemperatureReading,
            'temperature_reading',
            qos_profile
        )

        self.timer = self.create_timer(1.0 / freq_hz, self.timer_callback)

    def timer_callback(self):
        msg = TemperatureReading()
        msg.temperature = self.simulator.get_temperature()
        msg.unit = TemperatureReading.CELSIUS
        msg.sensor_id = self.sensor_id
        msg.timestamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published Temperature: {msg.temperature:.2f} {msg.unit} from {msg.sensor_id}'
        )


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Temperature Publisher Node')
    parser.add_argument('--frequency', type=float, default=1.0, help='Publishing frequency in Hz')
    parser.add_argument('--min_temp', type=float, default=35.0, help='Minimum temperature')
    parser.add_argument('--max_temp', type=float, default=38.0, help='Maximum temperature')
    parser.add_argument('--sensor_id', type=str, default='sensor_01', help='Sensor ID')

    parsed_args = parser.parse_args()

    node = TemperaturePublisher(
        parsed_args.frequency,
        parsed_args.min_temp,
        parsed_args.max_temp,
        parsed_args.sensor_id
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
