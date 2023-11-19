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


#define LED_PIN 13
#define STEP_PIN_MOTOR_RIGHT 3
#define DIR_PIN_MOTOR_RIGHT 6
#define STEP_PIN_MOTOR_LEFT 2
#define DIR_PIN_MOTOR_LEFT 5
#define MOTOR_POWER_SUPPLY_PIN 7

#define QUARTER_STEP 0.25
#define STEPS_PER_REVOLUTION int(1600 / QUARTER_STEP)
#define FEEDBACK_INTERVAL (STEPS_PER_REVOLUTION / 60)

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
rcl_subscription_t subscription_stop, subscription_move;
rcl_publisher_t publisher_status, publisher_log, publisher_feedback;
bool move_cancelled = false;

std_msgs__msg__Empty msg_empty;
raros_interfaces__msg__StepperMovement msg_stepper_movement;
raros_interfaces__msg__StepperStatus msg_status;
raros_interfaces__msg__StepperFeedback feedback_msg;

Stepper motor_left(STEPS_PER_REVOLUTION, STEP_PIN_MOTOR_LEFT, DIR_PIN_MOTOR_LEFT);
Stepper motor_right(STEPS_PER_REVOLUTION, STEP_PIN_MOTOR_RIGHT, DIR_PIN_MOTOR_RIGHT);

// ROS2 functions
// -----------------------------------------------------------------------------
void stop_callback(const void *msgin) {
    RCLC_UNUSED(msgin);
    stop_move();
}

void move_callback(const void *msgin) {
    const auto *movement_msg = (const raros_interfaces__msg__StepperMovement *) msgin;
    move(movement_msg->left, movement_msg->right);
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
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_init(&executor_stop, &support.context, 1, &allocator));

    // add subscription to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscription_move, &msg_stepper_movement, &move_callback,
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
    RCSOFTCHECK(rcl_publisher_fini(&publisher_status, &node));
    RCSOFTCHECK(rcl_publisher_fini(&publisher_log, &node));
    RCSOFTCHECK(rcl_publisher_fini(&publisher_feedback, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rclc_executor_fini(&executor_stop));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

// Stepper functions
// -----------------------------------------------------------------------------
void toggle_power_supply(bool on) {
    digitalWrite(MOTOR_POWER_SUPPLY_PIN, on ? HIGH : LOW);
}

void stop_move() {
    digitalWrite(STEP_PIN_MOTOR_RIGHT, LOW);
    digitalWrite(STEP_PIN_MOTOR_LEFT, LOW);
    toggle_power_supply(false);

    move_cancelled = true;
    publish_status(false);
    publish_log("stopped motors");
}

void move(raros_interfaces__msg__StepperInstruction instruction_left,
          raros_interfaces__msg__StepperInstruction instruction_right) {
    publish_status(true);
    toggle_power_supply(true);
    move_cancelled = false;

    publish_log("moving left stepper " + String(instruction_left.steps) + " steps at " +
                String(instruction_left.speed) + " RPM");
    publish_log("moving right stepper " + String(instruction_right.steps) + " steps at " +
                String(instruction_right.speed) + " RPM");

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

    publish_feedback(0, 0);
    publish_log("finished movement");
    toggle_power_supply(false);
    publish_status(false);
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