#!/usr/bin/env python3

'''
author: Ruben Roberts
email:  rober331@csusm.edu
org:    RoboticsClub
date:   Fall 2023
'''

import pygame
import rospy
from geometry_msgs.msg import Twist

class MainController:
    def __init__(self):
        # initialize pygame and its joystick module
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # can now initialize the ros node
        rospy.init_node('main_controller')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("main_controller node initialized")
                    
    def robotLoop(self):
        while not rospy.is_shutdown():
            pygame.event.pump()  # Process joystick events
            
            # loop over joystick axes to determine which to use
            #for i in range(joystick.get_numaxes()):
            #    rospy.loginfo(f"Axis {i}: {joystick.get_axis(i)}")

            twist = Twist()
            
            # Left stick (L3) controls linear velocity (forward and backward)
            twist.linear.x = -self.joystick.get_axis(1)  # Invert the axis for correct direction
            
            # Right stick (R3) controls angular velocity
            twist.angular.z = self.joystick.get_axis(3)
            
            self.pub.publish(twist)
            
            rospy.loginfo('linear velocity (twist.linear.x): ' + str(twist.linear.x))
            rospy.loginfo('angular velocity (twist.angular.z): ' + str(twist.angular.z))

if __name__ == '__main__':
    try:
        main_controller = MainController()
        main_controller.robotLoop()
        
    except rospy.ROSInterruptException:
        pass