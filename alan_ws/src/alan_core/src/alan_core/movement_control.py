#! /home/sorozco/computer_vision/bin/python3
import rospy
import time
import RPi.GPIO as GPIO
from gpiozero import Robot, DigitalInputDevice
from std_msgs.msg import Float32
from enum import Enum


class Robot:

    def __init__(self):
        self._movement_control = MovementControl()

        # power variables
        self._power = 0
        self.max_power = 1
        self.max_steering_power = 0.4

    def apply_power(self, power):
        if (not (0 <= power <= 1)):
            print("power must be between a range of 0 and 1")
            return
        # map power from [0,1] to [0,max_power]
        self._power = self.remap(abs(power), 0, 1, 0, self.max_power)

        ## suppport for backwards movement
        if power < 0:
            self._power *= -1

        self._movement_control.set_speed(self._power,self._power)

    def apply_steering_power(self, steering_power):
        if (not (-1 <= steering_power <= 1)):
            print("steering power must be between a range of -1 and 1")
            return

        steering_power = self.remap(abs(steering_power),-1,1,-self.max_steering_power,self.max_steering_power)

        # support for backwards movement
        # TODO: redo this logic
        if self._power < 0:
            power *= -1

        right_power = self._power
        left_power = self._power

        # steering power > 0 : turn left
        if steering_power > 0:

            # right wheel may be too slow, if so, increase left instead
            if self._power - abs(steering_power) > 0:
                right_power -= abs(steering_power)
            else:
                # max speed is still 100
                left_power = min(self._power + abs(steering_power), 1)

        # steering power < 0 : turn right
        elif steering_power < 0:

            # left wheel may be too slow, if so, increase right instead
            if self._power - abs(steering_power) > 0:
                left_power -= abs(steering_power)
            else:
                # max speed is still 100
                right_power = min(self._power + abs(steering_power), 1)

        self._movement_control.set_speed(left_power,right_power)

    def remap(self, old_value, old_min, old_max, new_min, new_max):
        return (
            ((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)
        ) + new_min


## if you dont cleanup, you might run into the issue of Runtime error:Failed to add edge detection
#GPIO.cleanup()

class Encoder:

    def __init__(self,pin):
        GPIO.setmode(GPIO.BCM)
        self._pin = pin
        self._value = 0

        #setup pins
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin,GPIO.RISING,callback=self._increment)

    def _increment(self,garbage):
        self._value += 1

    def reset(self):
        self._value = 0

    @property
    def value(self):
        return self._value

class Motor:
    def __init__(self,pwm_pin,fr_pin,encoder_pin):
        print("Motor")

        GPIO.setmode(GPIO.BCM)

        self._fr_pin = fr_pin
        self._pwm_pin = pwm_pin
        self._encoder_pin = encoder_pin

        #setup encoder for feedback loop
        self._encoder = Encoder(encoder_pin)

        #setup pins
        GPIO.setup(self._fr_pin,GPIO.OUT)
        GPIO.setup(self._pwm_pin,GPIO.OUT)

        #setup pwm for motor speed control
        self._pwm =GPIO.PWM(self._pwm_pin,50) 

        # motor starts idle
        self._pwm.start(0)
        GPIO.output(self._fr_pin,GPIO.LOW)

    def set_speed(self,value):
        if (value >= 0):
            GPIO.output(self._fr_pin,GPIO.LOW)
            self._pwm.ChangeDutyCycle(value)
        else:
            GPIO.output(self._fr_pin,GPIO.HIGH)
            self._pwm.ChangeDutyCycle(100 + value)

LEFT_PWM = 2
LEFT_FR = 3
LEFT_ENCODER = 18

RIGHT_PWM = 4
RIGHT_FR = 14
RIGHT_ENCODER = 15

class MovementControl:

    def __init__(self):

        self._left_wheel = Motor(LEFT_PWM,LEFT_FR,LEFT_ENCODER)
        self._right_wheel = Motor(RIGHT_PWM,RIGHT_FR,RIGHT_ENCODER)

    def set_speed(self,left_speed,right_speed):
        self._left_wheel.set_speed(left_speed)
        self._right_wheel.set_speed(right_speed)
