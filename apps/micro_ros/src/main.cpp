#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Stepper.h>

#include <RoboStepper.h>
#include <main.h>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/string.h>
#include <raros_interfaces/msg/stepper_movement.h>
#include <raros_interfaces/msg/stepper_status.h>
#include <raros_interfaces/msg/stepper_feedback.h>


#define LED_PIN 13
#define STEP_PIN_MOTOR_RIGHT 3
#define DIR_PIN_MOTOR_RIGHT 6
#define STEP_PIN_MOTOR_LEFT 2
#define DIR_PIN_MOTOR_LEFT 5
#define MOTOR_POWER_SUPPLY_PIN 7

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor, executor_stop;
rcl_subscription_t subscription_stop, subscription_move;
rcl_publisher_t publisher_status, publisher_log, publisher_feedback;
bool move_cancelled = false;

std_msgs__msg__Empty msg_empty;
raros_interfaces__msg__StepperMovement msg_stepper_movement;
raros_interfaces__msg__StepperStatus msg_status;
raros_interfaces__msg__StepperFeedback feedback_msg;

RoboStepper roboStepper;
Stepper motor_left(STEPS_PER_REVOLUTION, STEP_PIN_MOTOR_LEFT, DIR_PIN_MOTOR_LEFT);
Stepper motor_right(STEPS_PER_REVOLUTION, STEP_PIN_MOTOR_RIGHT, DIR_PIN_MOTOR_RIGHT);

// ROS2 functions
// -----------------------------------------------------------------------------
[[noreturn]]
void error_loop() {
    while (true) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void stop_callback(const void *msgin) {
    RCLC_UNUSED(msgin);
    stopMoving();
}

void move_callback(const void *msgin) {
    const auto *movement_msg = (const raros_interfaces__msg__StepperMovement *) msgin;
    move(movement_msg->left, movement_msg->right);
}

void publishStatus(bool moving) {
    msg_status.moving = moving;
    RCSOFTCHECK(rcl_publish(&publisher_status, &msg_status, nullptr))
}

void publishLog(String msg) {
    RCSOFTCHECK(rcl_publish(&publisher_log, &msg, nullptr))
}

void publishFeedback(int remaining_steps_left, int remaining_steps_right) {
    feedback_msg.remaining_steps_left = remaining_steps_left;
    feedback_msg.remaining_steps_right = remaining_steps_right;
    RCSOFTCHECK(rcl_publish(&publisher_feedback, &feedback_msg, nullptr))
}

// Stepper functions
// -----------------------------------------------------------------------------
void togglePowerSupply(bool on) {
    digitalWrite(MOTOR_POWER_SUPPLY_PIN, on ? HIGH : LOW);
}

void stopMoving() {
    togglePowerSupply(false);
    digitalWrite(STEP_PIN_MOTOR_RIGHT, LOW);
    digitalWrite(STEP_PIN_MOTOR_LEFT, LOW);

    move_cancelled = true;
    publishStatus(false);
    publishLog("stopped motors");
}

void move(raros_interfaces__msg__StepperMove stepper_move_left,
          raros_interfaces__msg__StepperMove stepper_move_right) {
    togglePowerSupply(true);
    publishStatus(true);
    move_cancelled = false;

    publishLog("Moving left stepper: " + String(stepper_move_left.distance) + "m with speed " +
               String(stepper_move_left.speed) + "rev/s");
    publishLog("Moving right stepper: " + String(stepper_move_right.distance) + "m with speed " +
               String(stepper_move_right.speed) + "rev/s");

    int steps_right = roboStepper.distanceInMeterToSteps(stepper_move_right.distance);
    int steps_left = roboStepper.distanceInMeterToSteps(stepper_move_left.distance);
    int steps_right_remaining = steps_right;
    int steps_left_remaining = steps_left;

    motor_left.setSpeed(stepper_move_left.speed);
    motor_right.setSpeed(stepper_move_right.speed);

    // check if no new stop command was received before starting
    RCCHECK(rclc_executor_spin_some(&executor_stop, RCL_US_TO_NS(10)))

    while ((steps_left_remaining > 0 || steps_right_remaining > 0) && !move_cancelled) {
        if (steps_left_remaining % 100 == 1 || steps_right_remaining % 100 == 1) {
            RCCHECK(rclc_executor_spin_some(&executor_stop, RCL_US_TO_NS(10)))
            publishFeedback(steps_left_remaining, steps_right_remaining);
        }
        if (steps_left_remaining > 0) {
            steps_left_remaining -= 1;
            motor_left.step(1);
        }

        if (steps_right_remaining > 0) {
            steps_right_remaining -= 1;
            motor_right.step(-1);
        }
    }

    publishFeedback(0, 0);
    publishLog("Finished moving");
    togglePowerSupply(false);
    publishStatus(false);
}

// Arduino functions
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(STEP_PIN_MOTOR_LEFT, OUTPUT);
    pinMode(STEP_PIN_MOTOR_RIGHT, OUTPUT);
    pinMode(DIR_PIN_MOTOR_LEFT, OUTPUT);
    pinMode(DIR_PIN_MOTOR_RIGHT, OUTPUT);
    pinMode(MOTOR_POWER_SUPPLY_PIN, OUTPUT);

    // wait for the serial port to be initialized
    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator))

    // create node
    RCCHECK(rclc_node_init_default(&node, "arduino_stepper", "raros", &support))

    // create subscriptions
    RCCHECK(rclc_subscription_init_default(
            &subscription_stop,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
            "arduino_stepper/stop"))

    RCCHECK(rclc_subscription_init_default(
            &subscription_move,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperMovement),
            "arduino_stepper/move"))

    // create publishers
    RCCHECK(rclc_publisher_init_default(
            &publisher_status,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperStatus),
            "arduino_stepper/status"))

    RCCHECK(rclc_publisher_init_best_effort(
            &publisher_log,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "arduino_stepper/log"))

    RCCHECK(rclc_publisher_init_best_effort(
            &publisher_feedback,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperFeedback),
            "arduino_stepper/feedback"))

    // create executors
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator))
    RCCHECK(rclc_executor_init(&executor_stop, &support.context, 1, &allocator))

    // add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscription_move, &msg_stepper_movement, &move_callback,
                                           ON_NEW_DATA))
    RCCHECK(rclc_executor_add_subscription(&executor_stop, &subscription_stop, &msg_empty, &stop_callback,
                                           ON_NEW_DATA))

    publishLog("node started successfully");
}

void loop() {
    delay(100);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)))
}