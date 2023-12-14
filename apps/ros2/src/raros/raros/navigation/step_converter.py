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

    def __angle_to_steps_for_rotation(self, angle_degrees):
        angle_radians = math.radians(angle_degrees)
        steps = int((angle_radians * self.wheel_distance * self.steps_per_revolution) / (self.wheel_circumference * 2))
        return steps

    def angle_to_steps_for_rotation(self, angle_degrees, direction: Direction):
        steps = self.__angle_to_steps_for_rotation(angle_degrees)
        if direction.value == Direction.DIRECTION_LEFT:
            return steps * -1, steps
        else:
            return steps, steps * -1

    def steps_to_angle(self, steps):
        angle_radians = steps / self.steps_per_revolution * self.wheel_circumference * 2 / self.wheel_distance
        return math.degrees(angle_radians)

    def angle_to_steps_for_turn(self, angle_degrees, direction: Direction):
        steps = 2 * self.__angle_to_steps_for_rotation(angle_degrees)
        if direction.value == Direction.DIRECTION_LEFT:
            return 0, steps
        else:
            return steps, 0

    def angle_and_radius_to_steps_for_turn(self, angle_degrees, radius, direction: Direction):
        def calculate_steps(radius, sign):
            adjusted_radius = radius + sign * self.wheel_distance / 2
            arc_length = self.calculate_circle_arc(adjusted_radius, angle_degrees)
            return self.distance_to_steps(arc_length, direction)

        if direction.value == Direction.DIRECTION_LEFT:
            return calculate_steps(radius, -1), calculate_steps(radius, 1)
        else:
            return calculate_steps(radius, 1), calculate_steps(radius, -1)

    @staticmethod
    def calculate_circle_arc(radius, angle_degrees):
        angle_radians = math.radians(angle_degrees)
        arc_length = angle_radians * radius
        return arc_length

    @staticmethod
    def turning_steps_to_speed(steps_left, steps_right, speed, direction: Direction):
        if direction.value == Direction.DIRECTION_LEFT:
            factor = steps_right / steps_left
            return int(speed / factor), speed
        else:
            factor = steps_left / steps_right
            return speed, int(speed / factor)
