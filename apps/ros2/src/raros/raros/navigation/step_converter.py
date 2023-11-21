import math

from raros_interfaces.msg import Direction


class Converter:
    def __init__(self, steps_per_revolution=6400, wheel_radius=0.0421, wheel_distance=0.22):
        self.steps_per_revolution = steps_per_revolution
        self.wheel_distance = wheel_distance
        self.wheel_circumference = wheel_radius * 2 * math.pi

    def distance_to_steps(self, distance, direction: Direction):
        steps = distance / self.wheel_circumference * self.steps_per_revolution
        if direction.value == Direction.DIRECTION_BACKWARD:
            steps *= -1
        return int(steps)

    def steps_to_distance(self, steps):
        return steps / self.steps_per_revolution * self.wheel_circumference

    def angle_to_steps_for_rotation(self, angle_degrees, direction: Direction):
        angle_radians = math.radians(angle_degrees)
        steps = int((angle_radians * self.wheel_distance * self.steps_per_revolution) / (self.wheel_circumference * 2))

        if direction.value == Direction.DIRECTION_RIGHT:
            return steps, steps * -1
        else:
            return steps * -1, steps

    def steps_to_angle(self, steps):
        angle_radians = steps / self.steps_per_revolution * self.wheel_circumference * 2 / self.wheel_distance
        return math.degrees(angle_radians)
