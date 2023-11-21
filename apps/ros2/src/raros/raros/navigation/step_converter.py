from raros_interfaces.msg import Direction


# TODO actually convert m to steps
def distance_to_steps(distance, direction: Direction):
    steps = distance * 1600 * 16
    # TODO move direction to dedicated message
    if direction.value == Direction.DIRECTION_BACKWARD:
        steps *= -1
    return int(steps)


def steps_to_distance(steps):
    return steps / 1600 / 16
