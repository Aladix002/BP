#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int32.h>

// Minimalny micro-ROS priklad pre ESP32 vetvu:
// - subscriber: /cmd_vel (geometry_msgs/Twist)
// - publisher:  /imu a /esp32/heartbeat
// POZN: Je to startovaci skeleton. Funkcie readImu* treba napojit na realny HW.

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_subscription_t sub_cmd_vel;
static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_heartbeat;
static rcl_timer_t timer_pub;

static geometry_msgs__msg__Twist cmd_vel_msg;
static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__UInt32 heartbeat_msg;

static volatile float target_linear_x = 0.0f;
static volatile float target_angular_z = 0.0f;
static uint32_t heartbeat_counter = 0;

// --- HW hooky (doplni sa podla konkretnej dosky/senzora) ---
static void applyMotorCommand(float linear_x, float angular_z) {
  // TODO: mapovanie na pohon ESP32 vetvy (H-most/PID firmware)
  (void)linear_x;
  (void)angular_z;
}

static float readImuYawRate() {
  // TODO: nacitanie gyro Z z IMU
  return 0.0f;
}

static float readImuAccelX() {
  // TODO: nacitanie akceleracie X
  return 0.0f;
}

static float readImuAccelY() {
  // TODO: nacitanie akceleracie Y
  return 0.0f;
}

static float readImuAccelZ() {
  // TODO: nacitanie akceleracie Z
  return 9.81f;
}

// --- ROS callbacky ---
static void onCmdVel(const void *msg_in) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  target_linear_x = (float)msg->linear.x;
  target_angular_z = (float)msg->angular.z;
  applyMotorCommand(target_linear_x, target_angular_z);
}

static void onTimer(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }

  imu_msg.angular_velocity.z = readImuYawRate();
  imu_msg.linear_acceleration.x = readImuAccelX();
  imu_msg.linear_acceleration.y = readImuAccelY();
  imu_msg.linear_acceleration.z = readImuAccelZ();
  rcl_publish(&pub_imu, &imu_msg, NULL);

  heartbeat_counter++;
  heartbeat_msg.data = heartbeat_counter;
  rcl_publish(&pub_heartbeat, &heartbeat_msg, NULL);
}

void setup() {
  // Priklad pre serial transport (agent na /dev/ttyUSB*):
  // napr. micro_ros_agent serial --dev /dev/ttyUSB0 -v6
  set_microros_serial_transports(Serial);
  Serial.begin(115200);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "esp32_micro_ros_node", "", &support);

  rclc_subscription_init_default(
      &sub_cmd_vel,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");

  rclc_publisher_init_default(
      &pub_imu,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu");

  rclc_publisher_init_default(
      &pub_heartbeat,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "/esp32/heartbeat");

  // 50 ms => 20 Hz
  rclc_timer_init_default(&timer_pub, &support, RCL_MS_TO_NS(50), onTimer);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(
      &executor, &sub_cmd_vel, &cmd_vel_msg, &onCmdVel, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer_pub);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}
