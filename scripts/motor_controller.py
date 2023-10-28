#!/usr/bin/env python3

'''
author: Ruben Roberts
email:  rober331@csusm.edu
org:    RoboticsClub
date:   Fall 2023
'''

import rospy
from geometry_msgs.msg import Twist
from motor import Motor
from direction import Direction

MOTOR1_ENABLE_PIN = 25
MOTOR1_IN1_PIN = 24
MOTOR1_IN2_PIN = 23

MOTOR2_ENABLE_PIN = 22
MOTOR2_IN1_PIN = 27
MOTOR2_IN2_PIN = 17

class MotorController:
    def __init__(self):
        # initialize this ros node as motor_controller and subscribe to the /cmd_vel topic
        rospy.init_node('motor_controller')
        rospy.Subscriber('/cmd_vel', Twist, self.callback)
        rospy.loginfo("motor_controller node initialized")
        
        # create motors and assign respective enable, in1, and in2 to each
        self.leftMotors = Motor(MOTOR1_ENABLE_PIN, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN)
        self.rightMotors = Motor(MOTOR2_ENABLE_PIN, MOTOR2_IN1_PIN, MOTOR2_IN2_PIN)

    def callback(self, data):
        direction = Direction.FORWARD if data.linear.x > 0 else Direction.REVERSE
        leftDutyCycle = rightDutyCycle = 0 if data.linear.x == 0 else abs(data.linear.x) * 100
        
        # need to adjust the duty cycle of each set of motors
        # if data.angular.z indicates need to turn
        if data.angular.z < 0:
            leftDutyCycle = abs(data.angular.z) * 0.25 * leftDutyCycle
            rospy.loginfo("turning left with duty cycle of " + str(leftDutyCycle))
        elif data.angular.z > 0:
            rightDutyCycle = abs(data.angular.z) * 0.25 * rightDutyCycle
            rospy.loginfo("turning right with duty cycle of " + str(rightDutyCycle))
        
        self.leftMotors.update(direction, leftDutyCycle)
        self.rightMotors.update(direction, rightDutyCycle)

if __name__ == '__main__':
    try:
        motor_controller = MotorController()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        motor_controller.leftMotors.cleanup()
        motor_controller.rightMotors.cleanup()
