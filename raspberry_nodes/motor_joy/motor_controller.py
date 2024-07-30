#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        
        # Subscriber to cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        
        # Define variables to store linear and angular velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
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

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.MOTOR1_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR1_DIR, GPIO.OUT)
        GPIO.setup(self.MOTOR2_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR2_DIR, GPIO.OUT)
        GPIO.setup(self.MOTOR3_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR3_DIR, GPIO.OUT)
        GPIO.setup(self.MOTOR4_PWM, GPIO.OUT)
        GPIO.setup(self.MOTOR4_DIR, GPIO.OUT)

        # Setup PWM
        self.pwm1 = GPIO.PWM(self.MOTOR1_PWM, 100)
        self.pwm2 = GPIO.PWM(self.MOTOR2_PWM, 100)
        self.pwm3 = GPIO.PWM(self.MOTOR3_PWM, 100)
        self.pwm4 = GPIO.PWM(self.MOTOR4_PWM, 100)

        self.pwm1.start(0)
        self.pwm2.start(0)
        self.pwm3.start(0)
        self.pwm4.start(0)

    def cmd_vel_callback(self, msg):
        # Callback function for cmd_vel messages
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        rospy.loginfo(f"Linear Vel: {self.linear_vel}, Angular Vel: {self.angular_vel}")

    def publish_motor_speeds(self):
        # Calculate motor speeds based on linear and angular velocities
        linear_velocity_fl = self.linear_vel + (self.angular_vel * self.robot_radius)
        linear_velocity_fr = self.linear_vel + (self.angular_vel * self.robot_radius)
        linear_velocity_rl = self.linear_vel + (self.angular_vel * self.robot_radius)
        linear_velocity_rr = self.linear_vel + (self.angular_vel * self.robot_radius)
        
        # Clip velocities to stay within the maximum speed
        linear_velocity_fl = min(max(linear_velocity_fl, -self.max_speed), self.max_speed)
        linear_velocity_fr = min(max(linear_velocity_fr, -self.max_speed), self.max_speed)
        linear_velocity_rl = min(max(linear_velocity_rl, -self.max_speed), self.max_speed)
        linear_velocity_rr = min(max(linear_velocity_rr, -self.max_speed), self.max_speed)
        
        # Convert linear velocities to motor speeds
        motor_speed_fl = int(linear_velocity_fl / self.max_speed * self.max_motor_speed)
        motor_speed_fr = int(linear_velocity_fr / self.max_speed * self.max_motor_speed)
        motor_speed_rl = int(linear_velocity_rl / self.max_speed * self.max_motor_speed)
        motor_speed_rr = int(linear_velocity_rr / self.max_speed * self.max_motor_speed)

        # Set motor directions and speeds
        self.set_motor_speed(self.pwm1, self.MOTOR1_DIR, motor_speed_fl)
        self.set_motor_speed(self.pwm2, self.MOTOR2_DIR, motor_speed_fr)
        self.set_motor_speed(self.pwm3, self.MOTOR3_DIR, motor_speed_rl)
        self.set_motor_speed(self.pwm4, self.MOTOR4_DIR, motor_speed_rr)

        rospy.loginfo(f"Motor Speeds - FL: {motor_speed_fl}, FR: {motor_speed_fr}, RL: {motor_speed_rl}, RR: {motor_speed_rr}")


    def set_motor_speed(self, pwm, dir_pin, speed):
        GPIO.output(dir_pin, GPIO.HIGH if speed >= 0 else GPIO.LOW)
        pwm.ChangeDutyCycle(min(max(abs(speed), 0), 100))  # Scale speed to PWM duty cycle (0-100)

    def run(self):
        # Main loop to continuously publish motor speeds
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_motor_speeds()
            rate.sleep()

    def cleanup(self):
        # Stop PWM and cleanup GPIO
        self.pwm1.stop()
        self.pwm2.stop()
        self.pwm3.stop()
        self.pwm4.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    motor_controller = MotorController()
    try:
        motor_controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_controller.cleanup()

