from enum import Enum


class AvoidanceState(Enum):
    STOPPED = 'stopped'
    FORWARD = 'forward'
    TURN_LEFT = 'turn_left'
    TURN_RIGHT = 'turn_right'


def next_state(
    current: AvoidanceState,
    *,
    front: str,
    left: str,
    right: str,
    enabled: bool = False,
) -> AvoidanceState:
    """Choose the next state from the latest sector statuses."""
    if not enabled:
        return AvoidanceState.STOPPED
    if front == 'clear':
        return AvoidanceState.FORWARD
    if front != 'blocked':
        return AvoidanceState.STOPPED

    if current is AvoidanceState.TURN_RIGHT and right == 'clear':
        return AvoidanceState.TURN_RIGHT
    if left == 'clear':
        return AvoidanceState.TURN_LEFT
    if right == 'clear':
        return AvoidanceState.TURN_RIGHT
    return AvoidanceState.STOPPED
