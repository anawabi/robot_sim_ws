from math import nan, radians

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


class Vl53l0xRangePublisher(Node):
    def __init__(self, sensor):
        super().__init__('vl53l0x_range')
        self.sensor = sensor
        self.frame_id = self.declare_parameter('frame_id', 'front_range_link').value
        self.publisher = self.create_publisher(Range, '/range', qos_profile_sensor_data)
        self.create_timer(0.1, self.publish_range)

    def publish_range(self):
        msg = Range()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = radians(25)
        msg.min_range = 0.05
        msg.max_range = 1.2
        try:
            distance = self.sensor.range / 1000.0
        except (OSError, RuntimeError) as exc:
            self.get_logger().warning(f'VL53L0X read failed: {exc}', throttle_duration_sec=2.0)
            distance = nan

        # An out-of-band value does not establish that the path is clear.
        msg.range = distance if msg.min_range <= distance <= msg.max_range else nan
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    import board
    from adafruit_vl53l0x import VL53L0X

    i2c = board.I2C()
    try:
        sensor = VL53L0X(i2c, address=0x29)
        rclpy.init(args=args)
        try:
            node = Vl53l0xRangePublisher(sensor)
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
    finally:
        i2c.deinit()


if __name__ == '__main__':
    main()
