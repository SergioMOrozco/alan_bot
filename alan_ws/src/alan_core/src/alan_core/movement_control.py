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
        self.power = 0
        self._right_power = 0
        self._left_power = 0
        self.max_power = max_power
        self.max_steering_power = max_steering_power

    def apply_power(self, power):
        # map power from [0,1] to [0,max_power]
        self.power = self.remap(abs(power), 0, 1, 0, self.max_power)

        ## suppport for backwards movement
        if power < 0:
            self.power *= -1

        self._right_power = self.power
        self._left_power = self.power

        self.publish_left_wheel_command()
        self.publish_right_wheel_command()

    def apply_steering_power(self, steering_power):
        # map steering power from [-1,1] to [0,1]
        power = self.remap(abs(steering_power), 0, 1, 0, self.max_steering_power)

        # support for backwards movement
        if self.power < 0:
            power *= -1

        # steering power > 0 : turn right
        if steering_power > 0:

            # right wheel may be too slow, if so, increase left instead
            if self.power - power > 0:
                self._right_power = self.power - power
            else:
                # max speed is still 1.0
                self._left_power = min(self.power + power, 1.0)

        # steering power < 0 : turn left
        elif steering_power < 0:

            # left wheel may be too slow, if so, increase right instead
            if self.power - power > 0:
                self._left_power = self.power - power
            else:
                # max speed is still 1.0
                self._right_power = min(self.power + power, 1.0)

        else:
            self._left_power = self.power
            self._right_power = self.power

        self.publish_right_wheel_command()
        self.publish_left_wheel_command()

    def remap(self, old_value, old_min, old_max, new_min, new_max):
        return (
            ((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)
        ) + new_min

    def get_right_power(self):
        return self._right_power

    def get_left_power(self):
        return self._left_power


## if you dont cleanup, you might run into the issue of Runtime error:Failed to add edge detection
#GPIO.cleanup()





#try:
#    for i in range(10):
#        p.ChangeDutyCycle(i)
#        time.sleep(0.02)
#    for i in range(10):
#        p.ChangeDutyCycle(10 - i)
#        time.sleep(0.02)
#
#    GPIO.output(left_fr,GPIO.HIGH)
#
#    for i in range(101):
#        p.ChangeDutyCycle(100 - i)
#        time.sleep(0.02)
#    for i in range(101):
#        p.ChangeDutyCycle(i)
#        time.sleep(0.02)
#
#    p.ChangeDutyCycle(0)
#    GPIO.output(left_fr,GPIO.LOW)
#
#    for i in range(101):
#        q.ChangeDutyCycle(i)
#        time.sleep(0.02)
#    for i in range(101):
#        q.ChangeDutyCycle(100 - i)
#        time.sleep(0.02)
#
#    GPIO.output(right_fr,GPIO.HIGH)
#
#    for i in range(101):
#        q.ChangeDutyCycle(100 - i)
#        time.sleep(0.02)
#    for i in range(101):
#        q.ChangeDutyCycle(i)
#        time.sleep(0.02)
#
#    q.ChangeDutyCycle(0)
#    GPIO.output(right_fr,GPIO.LOW)
#
#except KeyboardInterrupt:
#    pass

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

    @speed.setter
    def speed(self,value):
        if (value >= 0):
            GPIO.output(self._fr_pin,GPIO.LOW)
            self._pwm.ChangeDutyCycle(value)
        else:
            GPIO.output(self._fr_pin,GPIO.HIGH)
            self._pwm.ChangeDutyCycle(100 - value)


class MovementControl:

    LEFT_PWM = 2
    LEFT_FR = 3
    RIGHT_PWM = 4
    RIGHT_FR = 14


    RIGHT_ENCODER = 15
    LEFT_ENCODER = 18
    def __init__(self):

        self._left_wheel = Motor(LEFT_PWM,LEFT_FR,LEFT_ENCODER)
        self._right_wheel = Motor(RIGHT_PWM,RIGHT_FR,RIGHT_ENCODER)

    def value(self,speed):
        if type(speed) is tuple and len(speed) == 2:
            self.left_wheel.speed = speed[0]









