#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import RPi.GPIO as GPIO

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)

        # Subscriber to cmd_vel and controller topics
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        # rospy.Subscriber('/controller', Int16, self.gripper_callback)

        # Define variables to store linear and angular velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        # self.gripper_state = 0

        # Define robot parameters
        self.wheel_radius = 0.12  # Example wheel radius in meters
        self.robot_radius = 0.5  # Example distance from the center of the robot to the wheels in meters
        self.max_speed = 1.0  # Example maximum speed in meters per second
        self.max_motor_speed = 255  # Example maximum motor speed

        # GPIO pin setup
        self.MOTOR1_PWM = 9
        self.MOTOR1_DIR = 8
        self.MOTOR2_PWM = 10
        self.MOTOR2_DIR = 11
        self.MOTOR3_PWM = 6
        self.MOTOR3_DIR = 7
        self.MOTOR4_PWM = 5
        self.MOTOR4_DIR = 4
        # self.GRIPMOTOR_PWM = 15
        # self.GRIPMOTOR_DIR = 13

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.MOTOR1_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR1_DIR, GPIO.OUT)
        GPIO.setup(self.MOTOR2_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR2_DIR, GPIO.OUT)
        GPIO.setup(self.MOTOR3_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR3_DIR, GPIO.OUT)
        GPIO.setup(self.MOTOR4_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR4_DIR, GPIO.OUT)
        # GPIO.setup(self.GRIPMOTOR_PWM, GPIO.OUT)
        # GPIO.setup(self.GRIPMOTOR_DIR, GPIO.OUT)

        # Setup PWM
        self.pwm1 = GPIO.PWM(self.MOTOR1_PWM, 100)
        self.pwm2 = GPIO.PWM(self.MOTOR2_PWM, 100)
        self.pwm3 = GPIO.PWM(self.MOTOR3_PWM, 100)
        self.pwm4 = GPIO.PWM(self.MOTOR4_PWM, 100)
        # self.pwm5 = GPIO.PWM(self.GRIPMOTOR_PWM, 100)

        self.pwm1.start(0)
        self.pwm2.start(0)
        self.pwm3.start(0)
        self.pwm4.start(0)
        self.pwm5.start(0)

    def cmd_vel_callback(self, msg):
        # Callback function for cmd_vel messages
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    # def gripper_callback(self, msg):
    #     # Callback function for gripper control messages
    #     rospy.loginfo(f"Received gripper command: {msg.data}")
    #     self.gripper_state = msg.data
    #     self.set_gripper_state(self.gripper_state)

    # def set_gripper_state(self, state):
    #     # Function to control gripper based on the state received
    #     if state == 8:
    #         rospy.loginfo("Gripper: Arm Down")
    #         GPIO.output(self.GRIPMOTOR_DIR, GPIO.LOW)
    #         self.pwm5.ChangeDutyCycle(100)  # Adjust PWM for arm down
    #         time.sleep(1)
    #     elif state == 7:
    #         rospy.loginfo("Gripper: Arm Up")
    #         GPIO.output(self.GRIPMOTOR_DIR, GPIO.HIGH)
    #         self.pwm5.ChangeDutyCycle(100)  # Adjust PWM for arm up
    #         time.sleep(1)
    #     elif state == 9:
    #         rospy.loginfo("Gripper: Stop")
    #         self.pwm5.ChangeDutyCycle(0)


    def publish_motor_speeds(self):
        # Calculate motor speeds based on linear and angular velocities
        linear_velocity_left = self.linear_vel + (self.angular_vel * self.robot_radius)
        linear_velocity_right = self.linear_vel - (self.angular_vel * self.robot_radius)

        # Clip velocities to stay within the maximum speed
        linear_velocity_left = min(max(linear_velocity_left, -self.max_speed), self.max_speed)
        linear_velocity_right = min(max(linear_velocity_right, -self.max_speed), self.max_speed)
        
        # Convert linear velocities to motor speeds
        motor_speed_fr = int(linear_velocity_right / self.max_speed * self.max_motor_speed)
        motor_speed_fl = int(-linear_velocity_left / self.max_speed * self.max_motor_speed)
        motor_speed_rl = int(linear_velocity_left / self.max_speed * self.max_motor_speed)
        motor_speed_rr = int(-linear_velocity_right / self.max_speed * self.max_motor_speed)

        # Set motor directions and speeds
        self.set_motor_speed(self.pwm1, self.MOTOR1_DIR, -motor_speed_fr)
        self.set_motor_speed(self.pwm2, self.MOTOR2_DIR, -motor_speed_fl)
        self.set_motor_speed(self.pwm3, self.MOTOR3_DIR, motor_speed_rl)
        self.set_motor_speed(self.pwm4, self.MOTOR4_DIR, motor_speed_rr)
        
        rospy.loginfo(f"Motor Speeds - FL: {motor_speed_fl}, FR: {motor_speed_fr}, RL: {motor_speed_rl}, RR: {motor_speed_rr}")

    def set_motor_speed(self, pwm, dir_pin, speed):
        GPIO.output(dir_pin, GPIO.HIGH if speed >= 0 else GPIO.LOW)
        pwm.ChangeDutyCycle(min(max(abs(speed), 0), 100))  # Scale speed to PWM duty cycle (0-100)

    def run(self):
        # Main loop to control motors
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_motor_speeds()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = MotorController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
