from collections import Counter
from math import inf, isfinite, isnan, radians, remainder, tau

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from robot_control.avoidance import AvoidanceState, next_state


FRONT_HALF_ANGLE = radians(30)
SIDE_LIMIT = radians(90)
OBSTACLE_DISTANCE_M = 0.5


def scan_sample_angle(scan: LaserScan, index: int) -> float:
    """Return the angle in radians of an indexed range sample."""
    return scan.angle_min + index * scan.angle_increment


def classify_scan_range(distance: float, range_min: float, range_max: float) -> str:
    """Classify a LaserScan sample using the ROS REP 117 conventions."""
    if distance == inf:
        return 'no_return'
    if distance == -inf:
        return 'too_close'
    if isnan(distance):
        return 'invalid'
    if range_min <= distance <= range_max:
        return 'valid'
    return 'out_of_range'


def group_scan_sectors(
    scan: LaserScan, front_half_angle: float = FRONT_HALF_ANGLE, side_limit: float = SIDE_LIMIT,
) -> dict[str, list[float]]:
    """Group raw ranges by angle relative to the scan frame's forward axis."""
    sectors = {'front': [], 'left': [], 'right': []}
    for index, distance in enumerate(scan.ranges):
        angle = scan_sample_angle(scan, index)
        if not isfinite(angle):
            continue

        angle = remainder(angle, tau)
        if -front_half_angle <= angle <= front_half_angle:
            sectors['front'].append(distance)
        elif front_half_angle < angle <= side_limit:
            sectors['left'].append(distance)
        elif -side_limit <= angle < -front_half_angle:
            sectors['right'].append(distance)

    return sectors


def scan_covers_sector(scan: LaserScan, lower: float, upper: float) -> bool:
    """Check that the scan footprint spans the sector at its sampling resolution."""
    step = abs(scan.angle_increment)
    if len(scan.ranges) < 2 or not 0.0 < step < upper - lower:
        return False

    first = scan_sample_angle(scan, 0)
    last = scan_sample_angle(scan, len(scan.ranges) - 1)
    if not (isfinite(first) and isfinite(last)):
        return False

    start = min(first, last) - step / 2
    span = len(scan.ranges) * step
    return span >= tau or (lower - start) % tau + (upper - lower) <= span


def sector_status(
    ranges: list[float], range_min: float, range_max: float, obstacle_distance: float,
    *, full_coverage: bool,
) -> str:
    """Assess the rays present in one sector against a distance threshold."""
    can_assess_clearance = (
        full_coverage
        and isfinite(range_min)
        and isfinite(range_max)
        and 0.0 <= range_min <= obstacle_distance <= range_max
    )
    for distance in ranges:
        category = classify_scan_range(distance, range_min, range_max)
        if category == 'too_close' or (category == 'valid' and distance <= obstacle_distance):
            return 'blocked'
        if category in ('invalid', 'out_of_range'):
            can_assess_clearance = False

    if ranges and can_assess_clearance:
        return 'clear'
    return 'insufficient_data'


def assess_scan(scan, obstacle_distance, front_half_angle, side_limit):
    bounds = {
        'front': (-front_half_angle, front_half_angle),
        'left': (front_half_angle, side_limit),
        'right': (-side_limit, -front_half_angle),
    }
    return {
        name: (len(ranges), sector_status(
            ranges, scan.range_min, scan.range_max, obstacle_distance,
            full_coverage=scan_covers_sector(scan, *bounds[name]),
        ))
        for name, ranges in group_scan_sectors(scan, front_half_angle, side_limit).items()
    }


class ScanListener(Node):
    """Report live LaserScan metadata for diagnostics."""

    def __init__(self):
        super().__init__('scan_listener')
        self.declare_parameter('preview_avoidance', False)
        self.preview_timeout = self.declare_parameter('scan_timeout_sec', 0.5).value
        if not isfinite(self.preview_timeout) or self.preview_timeout <= 0:
            raise ValueError('scan_timeout_sec must be finite and positive')
        self.avoidance_state = AvoidanceState.STOPPED
        self.last_scan = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.check_preview, clock=self.steady_clock)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.on_scan,
            qos_profile_sensor_data,
        )

    def check_preview(self):
        enabled = self.get_parameter('preview_avoidance').value
        if self.last_scan is None:
            return
        stale = (self.steady_clock.now() - self.last_scan).nanoseconds >= self.preview_timeout * 1e9
        if self.avoidance_state is not AvoidanceState.STOPPED and (stale or not enabled):
            self.avoidance_state = AvoidanceState.STOPPED
            reason = 'scan timeout' if stale else 'preview disabled'
            self.get_logger().info(f'preview_state=stopped ({reason})')

    def on_scan(self, scan: LaserScan) -> None:
        self.last_scan = self.steady_clock.now()
        stamp = scan.header.stamp
        sample_count = len(scan.ranges)
        angle_summary = 'no sample angles'
        if sample_count:
            first_angle = scan_sample_angle(scan, 0)
            last_angle = scan_sample_angle(scan, sample_count - 1)
            angle_summary = f'first_angle={first_angle:.6f} last_angle={last_angle:.6f} rad'

        counts = Counter(
            classify_scan_range(distance, scan.range_min, scan.range_max)
            for distance in scan.ranges
        )
        range_summary = ' '.join(
            f'{category}={counts[category]}'
            for category in ('valid', 'out_of_range', 'no_return', 'too_close', 'invalid')
        )
        sector_summaries = []
        sector_statuses = {}
        sectors = assess_scan(scan, OBSTACLE_DISTANCE_M, FRONT_HALF_ANGLE, SIDE_LIMIT)
        for name, (sample_count_in_sector, status) in sectors.items():
            sector_statuses[name] = status
            sector_summaries.append(f'{name}_samples={sample_count_in_sector} {name}_status={status}')

        preview_enabled = self.get_parameter('preview_avoidance').value
        self.avoidance_state = next_state(
            self.avoidance_state, enabled=preview_enabled, **sector_statuses,
        )
        sector_summary = ' '.join(sector_summaries)
        self.get_logger().info(
            f'Scan frame={scan.header.frame_id} '
            f'stamp={stamp.sec}.{stamp.nanosec:09d} '
            f'samples={sample_count} {angle_summary} {range_summary} {sector_summary} '
            f'preview_enabled={preview_enabled} preview_state={self.avoidance_state.value}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
