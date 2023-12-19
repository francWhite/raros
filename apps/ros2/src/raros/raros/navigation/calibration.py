import math
import os
from datetime import datetime
from typing import Optional

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from raros.navigation.step_converter import Converter
from raros_interfaces.msg import Direction, StepperMovement, StepperParameters
from rclpy.node import Node
from std_msgs.msg import Empty as EmptyMsg


class Calibration(Node):
    def __init__(self):
        super().__init__('calibration')
        self.move_publisher = self.create_publisher(StepperMovement, '/raros/arduino_stepper/move', 10)
        self.parameters_publisher = self.create_publisher(StepperParameters, '/raros/arduino_stepper/parameters', 10)
        self.stop_publisher = self.create_publisher(EmptyMsg, '/raros/arduino_stepper/stop', 10)

        config_path = os.path.join(get_package_share_directory('raros'), 'config')
        self.params_file_path = os.path.join(config_path, 'params.yaml')
        self.params_calibrated_file_path = os.path.join(config_path, 'params_calibrated.yaml')
        self.converter: Optional[Converter] = None

    def init_parameters(self):
        current_wheel_distance, current_wheel_radius, steps_per_revolution, micro_steps = self.get_current_params()
        print('Using the current parameter values as a base for the calibration:')
        print(f'wheel.distance={current_wheel_distance}, wheel.radius={current_wheel_radius}, '
              f'steps_per_revolution={steps_per_revolution}, micro_steps={micro_steps}\n')
        self.converter = Converter(steps_per_revolution, micro_steps, current_wheel_radius, current_wheel_distance)

        stepper_parameters = StepperParameters()
        stepper_parameters.steps_per_revolution = steps_per_revolution
        stepper_parameters.micro_steps = micro_steps
        stepper_parameters.hold_power = True
        self.parameters_publisher.publish(stepper_parameters)

    def cleanup(self):
        stepper_parameters = StepperParameters()
        stepper_parameters.hold_power = False
        self.parameters_publisher.publish(stepper_parameters)
        self.stop_publisher.publish(EmptyMsg())

    def start_calibration(self):
        calibrated_wheel_radius = self.calibrate_wheel_radius()
        calibrated_wheel_distance = self.calibrate_wheel_distance(calibrated_wheel_radius)

        print('writing calculated values to params_calibrated.yaml...')
        data = self.create_config(calibrated_wheel_distance, calibrated_wheel_radius)
        self.write_config(data)

        print('Calibration finished! Please restart the raros package to use the new parameters')

    def calibrate_wheel_radius(self):
        self.print_wheel_radius_calibration_instructions()
        print('Beginning calibration, please enter the distance to travel.')
        distance = self.read_float('Distance [m]: ')

        steps = self.move(distance)
        print('Starting movement...')
        print('Press enter when the robot has finished moving.')
        input()

        print('Please measure and enter the distance between the two marks.')
        measured_distance = self.read_float('Distance [m]: ')
        calculated_wheel_radius = self.calculate_actual_radius(measured_distance, steps)
        print(f'Calculated wheel radius: {calculated_wheel_radius} cm\n')
        return calculated_wheel_radius

    def calibrate_wheel_distance(self, wheel_radius):
        self.print_wheel_distance_calibration_instructions()

        print('Beginning calibration, press enter to start the first rotation.')
        input()
        actual_angle = 360

        while True:
            steps = self.rotate(actual_angle)
            print('Starting rotation...')
            print('Press enter when the robot has finished rotating.')
            input()

            print('Please measure and enter the difference in degrees between the two marks.')
            angle_delta = self.read_float('Difference [°]: ')
            if angle_delta == 0:
                break

            actual_angle -= angle_delta
            print('Continuing calibration, reset the robot to the start position '
                  'and press enter to start the next rotation.')
            input()

        calculated_wheel_distance = self.calculate_actual_wheel_distance(wheel_radius, steps)
        print(f'Calculated wheel distance: {calculated_wheel_distance} cm\n')
        return calculated_wheel_distance

    def move(self, distance):
        steps = self.converter.distance_to_steps(distance, Direction(value=Direction.DIRECTION_FORWARD))
        stepper_msg = StepperMovement()
        stepper_msg.left.steps = steps
        stepper_msg.left.speed = 30
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps
        stepper_msg.right.speed = 30
        stepper_msg.right.speed_start = stepper_msg.right.speed
        self.move_publisher.publish(stepper_msg)
        return steps

    def rotate(self, angle_degrees):
        steps_left, steps_right = self.converter.angle_to_steps_for_rotation(angle_degrees,
                                                                             Direction(value=Direction.DIRECTION_LEFT))
        stepper_msg = StepperMovement()
        stepper_msg.left.steps = steps_left
        stepper_msg.left.speed = 15
        stepper_msg.left.speed_start = stepper_msg.left.speed
        stepper_msg.right.steps = steps_right
        stepper_msg.right.speed = 15
        stepper_msg.right.speed_start = stepper_msg.right.speed
        self.move_publisher.publish(stepper_msg)
        return abs(steps_left)

    def calculate_actual_radius(self, distance_in_m, steps):
        radius_in_m = distance_in_m / (steps / self.converter.steps_per_revolution) / (2 * math.pi)
        radius_in_cm = radius_in_m * 100
        return round(radius_in_cm, 4)

    def calculate_actual_wheel_distance(self, wheel_radius, steps):
        actual_distance = (4 * math.pi * wheel_radius * steps) / (self.converter.steps_per_revolution * 2 * math.pi)
        return round(actual_distance, 4)

    @staticmethod
    def print_wheel_radius_calibration_instructions():
        print('Starting calibration of wheel radius with the following procedure:')
        print(' 1. Mark the start position of robot')
        print(' 2. Instruct robot to move forward for a certain distance')
        print(' 3. Mark the end position of robot')
        print(' 4. Measure the distance between the two marks')
        print(' 5. Enter the measured distance in the following prompt')
        print('Based on the measured distance, the wheel radius will be calculated.\n')

    @staticmethod
    def print_wheel_distance_calibration_instructions():
        print('Starting calibration of wheel distance with the following procedure:')
        print(' 1. Mark the start position of robot')
        print(' 2. Instruct robot to turn 360°')
        print(' 3. Mark the end position of robot')
        print(' 4. Determine the difference between the two marks in degrees')
        print(' 5. Enter the difference in degrees. If the robot has exactly reached the start position, '
              'enter zero, otherwise the difference in degrees (positive if the robot has turned too far, '
              'negative if the robot has turned too little)')
        print('This procedure will be repeated until the end position is the same as the start position.\n')

    @staticmethod
    def read_float(message):
        while True:
            try:
                return float(input(message))
            except ValueError:
                print('Please enter a valid float value.')

    def get_current_params(self):
        if not os.path.isfile(self.params_file_path):
            print(f'No parameters file found at {self.params_file_path}. '
                  'Please make sure that raros is installed correctly.')
            raise SystemExit

        with (open(self.params_file_path, 'r') as file):
            data = yaml.load(file, Loader=yaml.FullLoader)
            try:
                navigation_config = data['raros']['navigation']['ros__parameters']
                return (navigation_config['wheel']['distance'], navigation_config['wheel']['radius'],
                        navigation_config['steps_per_revolution'], navigation_config['micro_steps'])
            except KeyError:
                print(f'At least one of the following parameters is missing in {self.params_file_path}: '
                      'wheel.distance, wheel.radius, steps_per_revolution, micro_steps.\n'
                      'These values are required as a base for the calibration. Please check the file and try again.')
                raise SystemExit

    def write_config(self, data):
        comment = f'# created by calibration at {datetime.now()}\n'
        with open(self.params_calibrated_file_path, 'w') as file:
            file.write(comment)
            yaml.dump(data, file, default_flow_style=False)

    @staticmethod
    def create_config(wheel_distance: float, wheel_radius: float):
        return {
            'raros': {
                'navigation': {
                    'ros__parameters': {
                        'wheel': {
                            'distance': wheel_distance,
                            'radius': wheel_radius
                        }
                    }
                }
            }
        }


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Calibration()
        print('waiting for the node to get ready...\n')
        while node.stop_publisher.get_subscription_count() == 0:
            pass

        node.init_parameters()
        node.start_calibration()
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        raise e
