from raros_interfaces.action import Move


# TODO actually convert m to steps
def distance_to_steps(distance, direction):
    steps = distance * 1600 * 16
    # TODO move direction to dedicated message
    if direction == Move.Goal.DIRECTION_BACKWARD:
        steps *= -1
    return int(steps)


def steps_to_distance(steps):
    return steps / 1600 / 16
