#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <QuadratureEncoder.h>

// ------------------- Pins -------------------
#define MOTOR1_PWM 13
#define MOTOR1_DIR 22
#define MOTOR2_PWM 12
#define MOTOR2_DIR 24
#define MOTOR3_PWM 7
#define MOTOR3_DIR 26
#define MOTOR4_PWM 5
#define MOTOR4_DIR 28

#define metal_detector 4
#define alarm 53
#define magnet 6
#define buzzer 52
#define Grip_Dir 8
#define Grip_Speed 9

// ------------------- Motor parameters -------------------
const float wheel_radius = 0.12; // meters
const float robot_radius = 0.5;  // meters
const float max_speed = 1.0;     // m/s
const int max_motor_speed = 255;

// ------------------- Objects -------------------
Encoders leftEncoder(10, 11);
Encoders rightEncoder(2, 3);

// ------------------- Micro-ROS -------------------
rcl_publisher_t left_ticks_pub;
rcl_publisher_t right_ticks_pub;
rcl_publisher_t detection_pub;

std_msgs__msg__Int16 left_ticks_msg;
std_msgs__msg__Int16 right_ticks_msg;
std_msgs__msg__Bool alert_msg;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t controller_sub;
std_msgs__msg__Int16 controller_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator = rcl_get_default_allocator();

// ------------------- Global variables -------------------
unsigned long lastMilli = 0;
unsigned long counter = 0;
int metal = 0;
int previous_metal = 0;

// ------------------- Motor control function -------------------
void setMotorSpeed(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(speed), 0, 255));
}

// ------------------- Callbacks -------------------
void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear_vel = msg->linear.x;
  float angular_vel = msg->angular.z;

  float linear_velocity_left = linear_vel + (angular_vel * robot_radius);
  float linear_velocity_right = linear_vel - (angular_vel * robot_radius);

  linear_velocity_left = constrain(linear_velocity_left, -max_speed, max_speed);
  linear_velocity_right = constrain(linear_velocity_right, -max_speed, max_speed);

  int motor_speed_fr = int(linear_velocity_right / max_speed * max_motor_speed);
  int motor_speed_fl = int(-linear_velocity_left / max_speed * max_motor_speed);
  int motor_speed_rl = int(linear_velocity_left / max_speed * max_motor_speed);
  int motor_speed_rr = int(-linear_velocity_right / max_speed * max_motor_speed);

  setMotorSpeed(MOTOR1_PWM, MOTOR1_DIR, -motor_speed_fr);
  setMotorSpeed(MOTOR2_PWM, MOTOR2_DIR, -motor_speed_fl);
  setMotorSpeed(MOTOR3_PWM, MOTOR3_DIR, motor_speed_rl);
  setMotorSpeed(MOTOR4_PWM, MOTOR4_DIR, motor_speed_rr);
}

void controllerCallback(const void * msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  int gripper = msg->data;

  if (gripper == 11) {
    digitalWrite(magnet, LOW);
  } else if (gripper == 10) {
    digitalWrite(magnet, HIGH);
  } else if (gripper == 8) {
    digitalWrite(Grip_Dir, HIGH);
    analogWrite(Grip_Speed, 80);
  } else if (gripper == 7) {
    digitalWrite(Grip_Dir, LOW);
    analogWrite(Grip_Speed, 80);
  } else {
    digitalWrite(Grip_Dir, HIGH);
    analogWrite(Grip_Speed, 0);
  }
}

// ------------------- Setup -------------------
void setup() {
  // Initialize pins
  pinMode(MOTOR1_PWM, OUTPUT); pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT); pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT); pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT); pinMode(MOTOR4_DIR, OUTPUT);

  pinMode(metal_detector, INPUT);
  pinMode(alarm, OUTPUT);
  pinMode(magnet, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(Grip_Dir, OUTPUT);
  pinMode(Grip_Speed, OUTPUT);

  digitalWrite(alarm, HIGH);
  digitalWrite(magnet, HIGH);

  // Initialize Micro-ROS
  set_microros_transports(); // Serial transport
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "arduino_robot_node", "", &support);

  // Publishers
  rclc_publisher_init_default(
    &left_ticks_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "left_ticks"
  );
  rclc_publisher_init_default(
    &right_ticks_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "right_ticks"
  );
  rclc_publisher_init_default(
    &detection_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "detection"
  );

  // Subscribers
  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );
  rclc_subscription_init_default(
    &controller_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "controller"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &controller_sub, &controller_msg, &controllerCallback, ON_NEW_DATA);
}

// ------------------- Loop -------------------
void loop() {
  // Metal detection
  metal = digitalRead(metal_detector);
  if (metal == HIGH) {
    counter++;
    digitalWrite(alarm, LOW);
    digitalWrite(buzzer, HIGH);
  } else {
    counter = 0;
    digitalWrite(alarm, HIGH);
    digitalWrite(buzzer, LOW);
    if (previous_metal != metal) {
      previous_metal = 0;
      alert_msg.data = false;
      rcl_ret_t ret = rcl_publish(&detection_pub, &alert_msg, NULL);
      (void)ret; // Suppress unused variable warning
    }
  }

  if (counter > 7) {
    if (previous_metal != metal) {
      previous_metal = 1;
      alert_msg.data = true;
      rcl_ret_t ret = rcl_publish(&detection_pub, &alert_msg, NULL);
      (void)ret; // Suppress unused variable warning
    }
  }

  // Publish encoder counts every 50ms
  if (millis() - lastMilli > 50) {
    left_ticks_msg.data = leftEncoder.getEncoderCount();
    right_ticks_msg.data = rightEncoder.getEncoderCount();
    rcl_ret_t ret1 = rcl_publish(&left_ticks_pub, &left_ticks_msg, NULL);
    rcl_ret_t ret2 = rcl_publish(&right_ticks_pub, &right_ticks_msg, NULL);
    (void)ret1; // Suppress unused variable warning
    (void)ret2; // Suppress unused variable warning
    lastMilli = millis();
  }

  // Spin Micro-ROS executor with timeout (10ms = 10000 microseconds)
  rclc_executor_spin_some(&executor, 10000);
}
