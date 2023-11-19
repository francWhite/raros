//
// Created by francisco on 18.11.23.
//

#ifndef MICRO_ROS_MAIN_H
#define MICRO_ROS_MAIN_H

#endif //MICRO_ROS_MAIN_H

#include <raros_interfaces/msg/stepper_move.h>
#include <raros_interfaces/msg/detail/stepper_status__struct.h>
#include <Arduino.h>

void stop_move();
void move(raros_interfaces__msg__StepperMove stepper_move_left, raros_interfaces__msg__StepperMove stepper_move_right);
void publish_status(bool moving);
void publish_log(String msg);
void publish_feedback(int remaining_steps_left, int remaining_steps_right);
void toggle_power_supply(bool on);