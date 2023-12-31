#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Stepper.h>

#include <main.h>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/string.h>
#include <raros_interfaces/msg/stepper_movement.h>
#include <raros_interfaces/msg/stepper_status.h>
#include <raros_interfaces/msg/stepper_feedback.h>
#include <raros_interfaces/msg/stepper_parameters.h>


#define LED_PIN 13
#define STEP_PIN_MOTOR_RIGHT 3
#define DIR_PIN_MOTOR_RIGHT 6
#define STEP_PIN_MOTOR_LEFT 2
#define DIR_PIN_MOTOR_LEFT 5
#define MOTOR_POWER_SUPPLY_PIN 7

#define FEEDBACK_INTERVAL (steps_per_revolution / 60)
#define ACCELERATION_RANGE_IN_REVOLUTIONS 4
#define ACCELERATION_RANGE_IN_STEPS (ACCELERATION_RANGE_IN_REVOLUTIONS * steps_per_revolution)

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor, executor_stop;
rcl_subscription_t subscription_stop, subscription_move, subscription_turn, subscription_parameters;
rcl_publisher_t publisher_status, publisher_log, publisher_feedback;

std_msgs__msg__Empty msg_empty;
raros_interfaces__msg__StepperMovement msg_stepper_movement;
raros_interfaces__msg__StepperStatus msg_status;
raros_interfaces__msg__StepperFeedback feedback_msg;
raros_interfaces__msg__StepperParameters parameters_msg;

bool move_cancelled = false;
bool hold_power = false;
int steps_per_revolution = 1600 * 4;

// default initialization
Stepper motor_left(steps_per_revolution, STEP_PIN_MOTOR_LEFT, DIR_PIN_MOTOR_LEFT);
Stepper motor_right(steps_per_revolution, STEP_PIN_MOTOR_RIGHT, DIR_PIN_MOTOR_RIGHT);

// ROS2 functions
// -----------------------------------------------------------------------------
void stop_callback(const void *msgin) {
    RCLC_UNUSED(msgin);
    stop_move();
}

void parameters_callback(const void *msgin) {
    const auto *parameters = (const raros_interfaces__msg__StepperParameters *) msgin;
    publish_log("received params steps_per_revolution: " + String(parameters->steps_per_revolution) +
                ", micro_steps: " + String(parameters->micro_steps) +
                ", hold_power: " + String(parameters->hold_power));

    steps_per_revolution = parameters->steps_per_revolution * parameters->micro_steps;
    hold_power = parameters->hold_power;
    motor_left = Stepper(steps_per_revolution, STEP_PIN_MOTOR_LEFT, DIR_PIN_MOTOR_LEFT);
    motor_right = Stepper(steps_per_revolution, STEP_PIN_MOTOR_RIGHT, DIR_PIN_MOTOR_RIGHT);
}

void move_callback(const void *msgin) {
    const auto *movement_msg = (const raros_interfaces__msg__StepperMovement *) msgin;
    if (movement_msg->left.speed == movement_msg->left.speed_start &&
        movement_msg->right.speed == movement_msg->right.speed_start)
        move(movement_msg->left, movement_msg->right);
    else
        move_with_acceleration(movement_msg->left, movement_msg->right);
}

void turn_callback(const void *msgin) {
    const auto *movement_msg = (const raros_interfaces__msg__StepperMovement *) msgin;
    turn(movement_msg->left, movement_msg->right);
}

void publish_status(bool moving) {
    msg_status.moving = moving;
    RCSOFTCHECK(rcl_publish(&publisher_status, &msg_status, nullptr));
}

void publish_log(String msg) {
    RCSOFTCHECK(rcl_publish(&publisher_log, &msg, nullptr));
}

void publish_feedback(int remaining_steps_left, int remaining_steps_right) {
    feedback_msg.remaining_steps_left = remaining_steps_left;
    feedback_msg.remaining_steps_right = remaining_steps_right;
    RCSOFTCHECK(rcl_publish(&publisher_feedback, &feedback_msg, nullptr));
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, "arduino_stepper", "raros", &support));

    // create subscriptions
    RCCHECK(rclc_subscription_init_default(
            &subscription_stop,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
            "arduino_stepper/stop"));

    RCCHECK(rclc_subscription_init_default(
            &subscription_move,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperMovement),
            "arduino_stepper/move"));

    RCCHECK(rclc_subscription_init_default(
            &subscription_turn,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperMovement),
            "arduino_stepper/turn"));

    RCCHECK(rclc_subscription_init_default(
            &subscription_parameters,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperParameters),
            "arduino_stepper/parameters"));

    // create publishers
    RCCHECK(rclc_publisher_init_default(
            &publisher_status,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperStatus),
            "arduino_stepper/status"));

    RCCHECK(rclc_publisher_init_best_effort(
            &publisher_log,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "arduino_stepper/log"));

    RCCHECK(rclc_publisher_init_best_effort(
            &publisher_feedback,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(raros_interfaces, msg, StepperFeedback),
            "arduino_stepper/feedback"));

    // create executors
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_init(&executor_stop, &support.context, 1, &allocator));

    // add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscription_move, &msg_stepper_movement, &move_callback,
                                           ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscription_turn, &msg_stepper_movement, &turn_callback,
                                           ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscription_parameters, &parameters_msg, &parameters_callback,
                                           ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor_stop, &subscription_stop, &msg_empty, &stop_callback,
                                           ON_NEW_DATA));

    publish_log("node successfully connected to agent and ready");
    return true;
}

void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    RCSOFTCHECK(rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0));

    RCSOFTCHECK(rcl_subscription_fini(&subscription_stop, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscription_move, &node));
    RCSOFTCHECK(rcl_subscription_fini(&subscription_turn, &node));
    RCSOFTCHECK(rcl_publisher_fini(&publisher_status, &node));
    RCSOFTCHECK(rcl_publisher_fini(&publisher_log, &node));
    RCSOFTCHECK(rcl_publisher_fini(&publisher_feedback, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rclc_executor_fini(&executor_stop));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));

    digitalWrite(MOTOR_POWER_SUPPLY_PIN, LOW);
}

// Stepper functions
// -----------------------------------------------------------------------------
void toggle_power_supply(bool on) {
    if (on) {
        digitalWrite(MOTOR_POWER_SUPPLY_PIN, HIGH);
    } else if (!hold_power) {
        digitalWrite(MOTOR_POWER_SUPPLY_PIN, LOW);
    }
}

void stop_move() {
    digitalWrite(STEP_PIN_MOTOR_RIGHT, LOW);
    digitalWrite(STEP_PIN_MOTOR_LEFT, LOW);
    toggle_power_supply(false);

    move_cancelled = true;
    publish_status(false);
    publish_log("stopped motors");
}

void begin_movement(raros_interfaces__msg__StepperInstruction instruction_left,
                    raros_interfaces__msg__StepperInstruction instruction_right) {
    publish_status(true);
    toggle_power_supply(true);
    move_cancelled = false;

    publish_log("left stepper instruction = steps: " + String(instruction_left.steps) + ", speed: " +
                String(instruction_left.speed) + ", speed_start: " + String(instruction_left.speed_start));
    publish_log("right stepper instruction = steps: " + String(instruction_right.steps) + ", speed: " +
                String(instruction_right.speed) + ", speed_start: " + String(instruction_right.speed_start));
}

void end_movement() {
    publish_feedback(0, 0);
    publish_log("finished movement");
    toggle_power_supply(false);
    publish_status(false);
}

void move(raros_interfaces__msg__StepperInstruction instruction_left,
          raros_interfaces__msg__StepperInstruction instruction_right) {

    begin_movement(instruction_left, instruction_right);

    int steps_right_direction = instruction_right.steps > 0 ? 1 : -1;
    int steps_right_remaining = abs(instruction_right.steps);
    int steps_left_direction = instruction_left.steps > 0 ? 1 : -1;
    int steps_left_remaining = abs(instruction_left.steps);
    int steps_done = 0;

    motor_left.setSpeed(instruction_left.speed);
    motor_right.setSpeed(instruction_right.speed);

    while ((steps_left_remaining > 0 || steps_right_remaining > 0) && !move_cancelled) {
        if (steps_done++ % FEEDBACK_INTERVAL == 0) {
            rclc_executor_spin_some(&executor_stop, RCL_US_TO_NS(10));
            publish_feedback(steps_left_remaining, steps_right_remaining);
        }

        if (steps_left_remaining > 0) {
            steps_left_remaining -= 1;
            motor_left.step(1 * steps_left_direction);
        }

        if (steps_right_remaining > 0) {
            steps_right_remaining -= 1;
            motor_right.step(-1 * steps_right_direction);
        }
    }

    end_movement();
}

void move_with_acceleration(raros_interfaces__msg__StepperInstruction instruction_left,
                            raros_interfaces__msg__StepperInstruction instruction_right) {
    begin_movement(instruction_left, instruction_right);

    int right_direction = instruction_right.steps > 0 ? 1 : -1;
    int left_direction = instruction_left.steps > 0 ? 1 : -1;

    //when moving with acceleration, the left and right motor should move identically
    int steps = min(abs(instruction_left.steps), abs(instruction_right.steps));
    int speed_min = min(instruction_left.speed, instruction_right.speed);
    int speed_max = min(instruction_left.speed_start, instruction_right.speed_start);

    int speed_delta = speed_max - speed_min;
    int steps_per_acceleration_increase = ACCELERATION_RANGE_IN_STEPS / speed_delta;
    int end_of_acceleration = 0;
    int start_of_deceleration = 0;

    if (steps > ACCELERATION_RANGE_IN_STEPS * 2) {
        end_of_acceleration = ACCELERATION_RANGE_IN_STEPS;
        start_of_deceleration = steps - ACCELERATION_RANGE_IN_STEPS;
    } else {
        end_of_acceleration = steps / 2;
        start_of_deceleration = steps / 2;
    }

    int steps_remaining = steps;
    int steps_done = 0;
    int current_speed = speed_min;
    motor_left.setSpeed(current_speed);
    motor_right.setSpeed(current_speed);

    while (steps_remaining > 0 && !move_cancelled) {
        if (steps_done++ % FEEDBACK_INTERVAL == 0) {
            rclc_executor_spin_some(&executor_stop, RCL_US_TO_NS(10));
            publish_feedback(steps_remaining, steps_remaining);
        }

        if (steps_done % steps_per_acceleration_increase == 0) {
            if (steps_done <= end_of_acceleration) {
                current_speed++;
            }
            if (steps_done >= start_of_deceleration) {
                current_speed--;
            }

            motor_left.setSpeed(current_speed);
            motor_right.setSpeed(current_speed);
        }

        motor_left.step(1 * left_direction);
        motor_right.step(-1 * right_direction);
        steps_remaining--;
    }

    end_movement();
}

void turn(raros_interfaces__msg__StepperInstruction instruction_left,
          raros_interfaces__msg__StepperInstruction instruction_right) {

    begin_movement(instruction_left, instruction_right);

    bool left_turn = instruction_right.steps > instruction_left.steps;
    int steps_min = min(instruction_left.steps, instruction_right.steps);
    int steps_max = max(instruction_left.steps, instruction_right.steps);
    int steps_remaining = steps_min;
    int steps_done = 0;

    double factor = double(steps_max) / double(steps_min);
    double fraction_part = factor - int(factor);
    double accumulated_fractions = 0;

    motor_left.setSpeed(instruction_left.speed);
    motor_right.setSpeed(instruction_right.speed);

    while (steps_remaining > 0 && !move_cancelled) {
        if (steps_done++ % FEEDBACK_INTERVAL == 0) {
            rclc_executor_spin_some(&executor_stop, RCL_US_TO_NS(10));
            publish_feedback(steps_remaining, steps_remaining);
        }

        int steps_to_move = 1;
        accumulated_fractions += fraction_part;

        if (accumulated_fractions >= 1) {
            accumulated_fractions -= 1;
            steps_to_move += 1;
        }

        if (left_turn) {
            motor_left.step(1);
            motor_right.step(-steps_to_move);
        } else {
            motor_left.step(steps_to_move);
            motor_right.step(-1);
        }

        steps_remaining--;
    }

    end_movement();
}

// Arduino functions
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(LED_PIN, OUTPUT);
    pinMode(STEP_PIN_MOTOR_LEFT, OUTPUT);
    pinMode(STEP_PIN_MOTOR_RIGHT, OUTPUT);
    pinMode(DIR_PIN_MOTOR_LEFT, OUTPUT);
    pinMode(DIR_PIN_MOTOR_RIGHT, OUTPUT);
    pinMode(MOTOR_POWER_SUPPLY_PIN, OUTPUT);

    state = WAITING_AGENT;
}

void loop() {
    switch (state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500,
                               state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) {
                destroy_entities();
            };
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED
                                                                                        : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
                rclc_executor_spin_some(&executor_stop, RCL_MS_TO_NS(10));
            }
            break;
        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }

    digitalWrite(LED_PIN, state == AGENT_CONNECTED);
}