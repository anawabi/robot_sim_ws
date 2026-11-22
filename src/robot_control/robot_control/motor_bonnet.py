from math import isfinite

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range


COMMAND_TIMEOUT_NS = 300_000_000
RANGE_TIMEOUT_NS = 350_000_000
FORWARD_STOP_DISTANCE_M = 0.25
FRONT_RANGE_FRAME = 'front_range_link'
DEFAULT_MAX_THROTTLE = 0.25
MAX_BENCH_THROTTLE = 0.50
I2C_ERRORS = (OSError, RuntimeError)


def side_throttles(
    linear: float, yaw: float, max_throttle: float = DEFAULT_MAX_THROTTLE,
) -> tuple[float, float]:
    """Map forward velocity and yaw rate to capped open-loop left/right throttle."""
    if not (isfinite(linear) and isfinite(yaw)):
        return 0.0, 0.0

    forward = max(-1.0, min(1.0, linear / 0.15))
    turn = max(-1.0, min(1.0, yaw / 0.6))
    left, right = forward - turn, forward + turn
    scale = max_throttle / max(1.0, abs(left), abs(right))
    return left * scale, right * scale


class MotorBonnet(Node):
    def __init__(self, open_kit):
        super().__init__('motor_bonnet')
        self.declare_parameter('enabled', False)
        self.max_throttle = self.declare_parameter(
            'max_throttle',
            DEFAULT_MAX_THROTTLE,
            ParameterDescriptor(
                description='Open-loop throttle ceiling. Set to 0.50 only for lifted bench tests.',
                read_only=True,
            ),
        ).value
        if not 0.0 < self.max_throttle <= MAX_BENCH_THROTTLE:
            raise ValueError('max_throttle must be greater than 0.0 and at most 0.50')
        self.open_kit = open_kit
        self.right_motors = []
        self.left_motors = []
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.last_command = None
        self.last_clear_range = None
        self.forward_requested = False
        self.requested = (0.0, 0.0)
        self.applied = (0.0, 0.0)
        self.armed = False
        self.faulted = False
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 1)
        self.create_subscription(
            Range, '/range', self.on_range,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(0.05, self.control, clock=self.clock)

    def forward_is_clear(self, now):
        return (
            self.last_clear_range is not None
            and (now - self.last_clear_range).nanoseconds < RANGE_TIMEOUT_NS
        )

    def on_range(self, reading: Range):
        if (
            reading.header.frame_id == FRONT_RANGE_FRAME
            and reading.radiation_type == Range.INFRARED
            and isfinite(reading.range)
            and isfinite(reading.min_range)
            and isfinite(reading.max_range)
            and 0.0 <= reading.min_range <= FORWARD_STOP_DISTANCE_M < reading.range
            <= reading.max_range
        ):
            now = self.clock.now()
            if self.forward_requested and not self.forward_is_clear(now):
                self.last_command = None
            self.last_clear_range = now
        else:
            self.last_clear_range = None
            if self.forward_requested:
                self.last_command = None

    def on_command(self, command: Twist):
        if self.armed and self.get_parameter('enabled').value:
            now = self.clock.now()
            self.forward_requested = command.linear.x > 0.0
            if self.forward_requested and not self.forward_is_clear(now):
                self.last_command = None
                return
            self.requested = side_throttles(command.linear.x, command.angular.z, self.max_throttle)
            self.last_command = now

    def stop_motors(self):
        self.armed = False
        self.last_command = None
        self.forward_requested = False
        for motor in (*self.right_motors, *self.left_motors):
            try:
                motor.throttle = 0.0
            except I2C_ERRORS as exc:
                self.faulted = True
                self.get_logger().error(f'Could not stop a motor: {exc}', throttle_duration_sec=2.0)
        self.applied = (0.0, 0.0)

    def control(self):
        if self.faulted or not self.get_parameter('enabled').value:
            if self.armed or self.faulted:
                self.stop_motors()
            return

        try:
            if not self.armed:
                if not self.right_motors and not self.left_motors:
                    kit = self.open_kit()
                    # M1 front right, M2 rear right, M3 rear left, M4 front left.
                    self.right_motors.extend((kit.motor1, kit.motor2))
                    self.left_motors.extend((kit.motor3, kit.motor4))
                self.stop_motors()
                self.armed = not self.faulted
                return

            target = (0.0, 0.0)
            if self.last_command is not None:
                now = self.clock.now()
                if self.forward_requested and not self.forward_is_clear(now):
                    self.last_command = None
                elif (now - self.last_command).nanoseconds < COMMAND_TIMEOUT_NS:
                    target = self.requested
            if target != self.applied:
                left, right = target
                for motor in self.right_motors:
                    motor.throttle = right
                for motor in self.left_motors:
                    motor.throttle = left
                self.applied = target
        except I2C_ERRORS as exc:
            self.faulted = True
            self.get_logger().error(f'Motor Bonnet I2C failure: {exc}. Restart required.')
            self.stop_motors()


def main(args=None):
    i2c = None

    def open_kit():
        import board
        from adafruit_motorkit import MotorKit

        nonlocal i2c
        i2c = board.I2C()
        return MotorKit(i2c=i2c, address=0x60)

    rclpy.init(args=args)
    try:
        node = MotorBonnet(open_kit)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.stop_motors()
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
        if i2c is not None:
            i2c.deinit()


if __name__ == '__main__':
    main()
