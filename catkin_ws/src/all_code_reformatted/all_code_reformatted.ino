#include <QuadratureEncoder.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h> // For cmd_vel messages

// Motor PWM and Direction pins
#define MOTOR1_PWM 13
#define MOTOR1_DIR 22
#define MOTOR2_PWM 12
#define MOTOR2_DIR 24
#define MOTOR3_PWM 7
#define MOTOR3_DIR 26
#define MOTOR4_PWM 5
#define MOTOR4_DIR 28

// Sensor and output pins
#define metal_detector 4
#define alarm 53
#define magnet 6
#define buzzer 52
#define Grip_Dir 8
#define Grip_Speed 9

// Robot and motor parameters
const float wheel_radius = 0.12; // meters
const float robot_radius = 0.5; // meters
const float max_speed = 1.0; // m/s
const int max_motor_speed = 255;

// Encoder objects for left and right wheels
Encoders leftEncoder(10, 11);
Encoders rightEncoder(2, 3);

// ROS node handle
ros::NodeHandle nh;

// ROS publishers for encoder ticks and metal detection
std_msgs::Int16 left_ticks_count;
ros::Publisher left_ticks("left_ticks", &left_ticks_count);

std_msgs::Int16 right_ticks_count;
ros::Publisher right_ticks("right_ticks", &right_ticks_count);

std_msgs::Bool alert;
ros::Publisher detection("detection", &alert);

// Callback function for receiving velocity commands (cmd_vel) **Subscribers**

void cmdVelCallback(const geometry_msgs::Twist &msg) {
  // Extract linear and angular velocities from the message
  float linear_vel = msg.linear.x;
  float angular_vel = msg.angular.z;
  
  // Compute left and right wheel velocities
  float linear_velocity_left = linear_vel + (angular_vel * robot_radius);
  float linear_velocity_right = linear_vel - (angular_vel * robot_radius);

  // Constrain velocities to the maximum speed
  linear_velocity_left = constrain(linear_velocity_left, -max_speed, max_speed);
  linear_velocity_right = constrain(linear_velocity_right, -max_speed, max_speed);

  // Convert velocities to motor speeds
  int motor_speed_fr = int(linear_velocity_right / max_speed * max_motor_speed);
  int motor_speed_fl = int(-linear_velocity_left / max_speed * max_motor_speed);
  int motor_speed_rl = int(linear_velocity_left / max_speed * max_motor_speed);
  int motor_speed_rr = int(-linear_velocity_right / max_speed * max_motor_speed);

  // Set motor speeds for each motor
  setMotorSpeed(MOTOR1_PWM, MOTOR1_DIR, -motor_speed_fr);
  setMotorSpeed(MOTOR2_PWM, MOTOR2_DIR, -motor_speed_fl);
  setMotorSpeed(MOTOR3_PWM, MOTOR3_DIR, motor_speed_rl);
  setMotorSpeed(MOTOR4_PWM, MOTOR4_DIR, motor_speed_rr);
}

// ROS subscriber for velocity commands
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmdVelCallback);

// Callback function for gripper control based on received commands
void controllerCallback(const std_msgs::Int16 &gripper) {
  if (gripper.data == 11) {
    digitalWrite(magnet, LOW);
  } else if (gripper.data == 10) {
    digitalWrite(magnet, HIGH);
  } else if (gripper.data == 8) {
    digitalWrite(Grip_Dir, HIGH);
    analogWrite(Grip_Speed, 80);
  } else if (gripper.data == 7) {
    digitalWrite(Grip_Dir, LOW);
    analogWrite(Grip_Speed, 80);
  } else {
    digitalWrite(Grip_Dir, HIGH);
    analogWrite(Grip_Speed, 0);
  }
}

// ROS subscriber for gripper control commands
ros::Subscriber<std_msgs::Int16> controller_sub("controller", controllerCallback);

// Global variables for metal detection and timing
unsigned long lastMilli = 0;
unsigned long last_detected = 0;
int metal = 0;
int previous_metal = 0;
int counter = 0;

// Function to set motor speed based on PWM and direction
void setMotorSpeed(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(speed), 0, 255)); 
}

void setup() {
  // Initialize sensor and output pins
  pinMode(metal_detector, INPUT);
  pinMode(alarm, OUTPUT);
  pinMode(magnet, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // Initialize motor control pins
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_DIR, OUTPUT);
  pinMode(MOTOR4_PWM, OUTPUT);
  pinMode(MOTOR4_DIR, OUTPUT);

  // Initialize ROS node and advertise publishers
  nh.initNode();
  nh.advertise(left_ticks);
  nh.advertise(right_ticks);
  nh.advertise(detection);
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(controller_sub);

  // Initialize motor and sensor states
  digitalWrite(alarm, HIGH);
  digitalWrite(magnet, HIGH);
}

void loop() {

  // Metal detection and alarm handling
  metal = digitalRead(metal_detector);

  if (metal == HIGH) { // Metal detected
    counter++;
    digitalWrite(alarm, LOW);   // Activate alarm
    digitalWrite(buzzer, HIGH); // Sound buzzer
  } else {
    counter = 0;
    digitalWrite(alarm, HIGH);  // Deactivate alarm
    digitalWrite(buzzer, LOW);  // Silence buzzer
    if (previous_metal != metal) {
      previous_metal = 0;
      alert.data = false;
      detection.publish(&alert); // Publish no detection alert
    }
  }

  // Publish detection alert if metal is detected continuously
  if (counter > 7) {
    if (previous_metal != metal) {
      previous_metal = 1;
      alert.data = true;
      detection.publish(&alert);  // Publish detection alert
    }
  }

  // Publish encoder counts at regular intervals (every 50 ms)
  if (millis() - lastMilli > 50) {
    left_ticks_count.data = leftEncoder.getEncoderCount();
    right_ticks_count.data = rightEncoder.getEncoderCount();
    left_ticks.publish(&left_ticks_count);
    right_ticks.publish(&right_ticks_count);
    lastMilli = millis();
  }

  nh.spinOnce();  // Process incoming ROS messages
}
