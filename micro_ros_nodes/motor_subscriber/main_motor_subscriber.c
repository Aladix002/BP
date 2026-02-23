/**
 * Vzorový micro-ROS subscriber: príkaz pre motory (Float64MultiArray).
 * Určené pre ESP32 s micro-ROS (FreeRTOS). Po prijatí správy [left, right]
 * v rozsahu -1..1 aplikátor volá vlastnú funkciu na nastavenie motorov (PWM/H-bridge).
 *
 * Build: v rámci micro_ros_setup pre platformu esp32, alebo ESP-IDF s micro_ros_component.
 * Na hostovi musí bežať micro_ros_agent (serial alebo WiFi podľa konfigurácie).
 */

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <stdio.h>

#define RCCHECK(fn) do { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed: " #fn "\n"); return; } } while (0)
#define RCSOFTCHECK(fn) do { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Warning: " #fn "\n"); } } while (0)

rcl_subscription_t subscription;
std_msgs__msg__Float64MultiArray msg_motor;

void motor_cmd_callback(const void *msgin) {
    const std_msgs__msg__Float64MultiArray *msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    if (msg->data.size < 2) return;
    double left  = msg->data.data[0];
    double right = msg->data.data[1];
    /* Tu: volanie do vlastného kódu na nastavenie motorov (napr. PWM). */
    /* Napr.: set_motors(left, right); */
    (void)left;
    (void)right;
}

void app_micro_ros_init(void);
void app_micro_ros_run(void);

void app_micro_ros_run(void) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "motor_node", "", &support);

    rclc_subscription_init_default(&subscription, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "/motor_cmd");

    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscription, &msg_motor, &motor_cmd_callback, ON_NEW_DATA);

    while (1) {
        rclc_executor_spin_some(&executor, 100);
    }
}

/* app_micro_ros_init() sa volá pred app_micro_ros_run() a nastaví transport (serial/WiFi). */
/* Implementácia závisí od micro_ros_platform (esp32) – pozri oficiálne tutoriály. */
