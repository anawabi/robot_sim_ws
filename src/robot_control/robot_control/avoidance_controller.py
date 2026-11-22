from math import isfinite, radians
import signal

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from robot_control.avoidance import AvoidanceState, next_state
from robot_control.scan_listener import assess_scan


class AvoidanceController(Node):
    def __init__(self):
        super().__init__('avoidance_controller')
        self.declare_parameter('enabled', False)
        descriptor = ParameterDescriptor(read_only=True)
        defaults = [
            ('forward_speed_mps', 0.15),
            ('turn_speed_radps', 0.6),
            ('obstacle_distance_m', 0.5),
            ('front_half_angle_deg', 30.0),
            ('side_limit_deg', 90.0),
            ('scan_timeout_sec', 0.5),
            ('scan_frame', 'lidar_link'),
        ]
        settings = {
            name: self.declare_parameter(name, default, descriptor).value
            for name, default in defaults
        }
        if any(not isfinite(settings[name]) or settings[name] <= 0 for name, _ in defaults[:-1]):
            raise ValueError('Controller numeric parameters must be finite and positive')
        if not settings['front_half_angle_deg'] < settings['side_limit_deg'] <= 180.0:
            raise ValueError('Require front_half_angle_deg < side_limit_deg <= 180')

        self.forward_speed = settings['forward_speed_mps']
        self.turn_speed = settings['turn_speed_radps']
        self.obstacle_distance = settings['obstacle_distance_m']
        self.front_half_angle = radians(settings['front_half_angle_deg'])
        self.side_limit = radians(settings['side_limit_deg'])
        self.timeout_ns = settings['scan_timeout_sec'] * 1e9
        self.scan_frame = settings['scan_frame']
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.command_topic = self.resolve_topic_name('/cmd_vel')
        self.command_publisher = None
        self.last_scan = None
        self.sectors = None
        self.state = AvoidanceState.STOPPED

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self.control, clock=self.clock)

    def on_scan(self, scan):
        if scan.header.frame_id != self.scan_frame:
            self.last_scan = None
            return
        assessed = assess_scan(scan, self.obstacle_distance, self.front_half_angle, self.side_limit)
        self.sectors = {name: status for name, (_, status) in assessed.items()}
        self.last_scan = self.clock.now()

    def release_command_topic(self):
        if self.command_publisher is not None:
            if self.count_publishers(self.command_topic) == 1:
                self.command_publisher.publish(Twist())
                self.command_publisher.wait_for_all_acked(Duration(seconds=0.1))
            self.destroy_publisher(self.command_publisher)
            self.command_publisher = None
        self.state = AvoidanceState.STOPPED
        self.last_scan = None

    def control(self):
        if not self.get_parameter('enabled').value:
            if self.command_publisher is not None:
                self.release_command_topic()
            return

        publishers = self.count_publishers(self.command_topic)
        if publishers > int(self.command_publisher is not None):
            self.release_command_topic()
            self.get_logger().warning(
                f'Another publisher is using {self.command_topic}. Waiting for it to release control.',
                throttle_duration_sec=2.0,
            )
            return
        if self.command_publisher is None:
            self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
            return
        if publishers != 1:
            return

        state = AvoidanceState.STOPPED
        fresh = (
            self.last_scan is not None
            and (self.clock.now() - self.last_scan).nanoseconds < self.timeout_ns
        )
        if fresh:
            state = next_state(self.state, enabled=True, **self.sectors)
        if state is not self.state:
            self.get_logger().info(f'avoidance_state={state.value}')
        self.state = state

        command = Twist()
        if state is AvoidanceState.FORWARD:
            command.linear.x = self.forward_speed
        elif state is AvoidanceState.TURN_LEFT:
            command.angular.z = self.turn_speed
        elif state is AvoidanceState.TURN_RIGHT:
            command.angular.z = -self.turn_speed
        self.command_publisher.publish(command)


def main(args=None):
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, interrupt)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = AvoidanceController()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.release_command_topic()
            node.destroy_node()
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        rclpy.try_shutdown()
