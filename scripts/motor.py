#!/usr/bin/env python3

'''
author: Ruben Roberts
email:  rober331@csusm.edu
org:    RoboticsClub
date:   Fall 2023
'''

import RPi.GPIO as GPIO
from enum import Enum
from direction import Direction

class Motor:
    def __init__(self, enablePin, in1Pin, in2Pin):
        self.enablePin = enablePin
        self.in1Pin = in1Pin
        self.in2Pin = in2Pin
        self.temp1 = 1
        
        GPIO.setwarnings(False)

        # set the pin numbering mode...This uses BCM channel number instead of physical pin on board
        GPIO.setmode(GPIO.BCM)
        
        # set pins as output pins
        GPIO.setup(self.enablePin, GPIO.OUT)
        GPIO.setup(self.in1Pin, GPIO.OUT)
        GPIO.setup(self.in2Pin, GPIO.OUT)
        
        # turn the pins off
        GPIO.output(self.in1Pin, GPIO.LOW)
        GPIO.output(self.in2Pin, GPIO.LOW)

        # create a PWM object at 1000 Hz, attached to the enablePin
        self.pwmPin = GPIO.PWM(self.enablePin, 1000)
        
        # initialize the pwm signal to a duty cycle of 0% (off)
        self.pwmPin.start(0)

    def update(self, direction=Direction.FORWARD, dutyCycle=0):
        if direction is Direction.FORWARD:
            GPIO.output(self.in1Pin, GPIO.HIGH)
            GPIO.output(self.in2Pin, GPIO.LOW)
        else:
            GPIO.output(self.in1Pin, GPIO.LOW)
            GPIO.output(self.in2Pin, GPIO.HIGH)

        self.pwmPin.ChangeDutyCycle(dutyCycle)

    def cleanup(self):
        GPIO.cleanup()
