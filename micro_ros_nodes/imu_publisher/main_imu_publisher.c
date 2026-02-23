/**
 * Vzorový micro-ROS publisher: IMU dáta (sensor_msgs/msg/Imu).
 * Určené pre ESP32 s micro-ROS (FreeRTOS). Periodicky číta akcelerometer/gyro
 * (napr. MPU6050 cez I2C), vyplní správu a publikuje na topic /imu.
 *
 * Build: v rámci micro_ros_setup pre platformu esp32, alebo ESP-IDF s micro_ros_component.
 * Na hostovi musí bežať micro_ros_agent.
 */

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RCCHECK(fn) do { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Failed: " #fn "\n"); return; } } while (0)
#define RCSOFTCHECK(fn) do { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { printf("Warning: " #fn "\n"); } } while (0)

rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg_imu;

void app_micro_ros_init(void);
void app_micro_ros_run(void);

/* Placeholder: načítanie surových dát z IMU (MPU6050 alebo iný). */
static void read_imu_raw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    (void)ax; (void)ay; (void)az;
    (void)gx; (void)gy; (void)gz;
    /* napr. i2c read z MPU6050, konverzia na m/s² a rad/s */
}

void app_micro_ros_run(void) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "imu_node", "", &support);

    rclc_publisher_init_default(&publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu");

    msg_imu.header.frame_id = "imu_link";

    while (1) {
        float ax, ay, az, gx, gy, gz;
        read_imu_raw(&ax, &ay, &az, &gx, &gy, &gz);

        msg_imu.linear_acceleration.x = (double)ax;
        msg_imu.linear_acceleration.y = (double)ay;
        msg_imu.linear_acceleration.z = (double)az;
        msg_imu.angular_velocity.x = (double)gx;
        msg_imu.angular_velocity.y = (double)gy;
        msg_imu.angular_velocity.z = (double)gz;
        /* orientation (quaternion) môže ostať 0 alebo doplniť fúziou ak je implementovaná */

        RCSOFTCHECK(rcl_publish(&publisher, &msg_imu, NULL));

        /* typicky 20–50 Hz; na FreeRTOS: vTaskDelay(pdMS_TO_TICKS(50)); */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* app_micro_ros_init() – nastavenie transportu (serial/WiFi) podľa micro_ros_platform. */
