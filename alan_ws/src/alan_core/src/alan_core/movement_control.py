#! /home/sorozco/computer_vision/bin/python3
import rospy
from threading import Thread
import time
import RPi.GPIO as GPIO
from gpiozero import Robot, DigitalInputDevice
from std_msgs.msg import Float32
from enum import Enum

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

    SAMPLETIME = 1

    def __init__(self,pwm_pin,fr_pin,encoder_pin = 0, use_feedback_loop = False):
        self._state = 0
        self._control_effort = 0
        self._set_point = 0

        GPIO.setmode(GPIO.BCM)

        self._fr_pin = fr_pin
        self._pwm_pin = pwm_pin

        #setup pins
        GPIO.setup(self._fr_pin,GPIO.OUT)
        GPIO.setup(self._pwm_pin,GPIO.OUT)

        #setup pwm for motor speed control
        self._pwm =GPIO.PWM(self._pwm_pin,50) 

        # motor starts idle
        self._pwm.start(0)
        GPIO.output(self._fr_pin,GPIO.LOW)

        if (use_feedback_loop):
            self._kp = 0.3
            self._ki = 0
            self._kd = 0.15

            self._encoder_pin = encoder_pin

            #setup encoder for feedback loop
            self._encoder = Encoder(encoder_pin)
            self._tune()

    def _tune(self):
        x = Thread(target=self._tuning_thread)
        x.start()

    def _tuning_thread(self):

        prev_error = 0
        sum_error = 0
        while (True):

            rpm  = 60 * (self._encoder.value / 384.0) / 0.25

            self._state = self.remap(rpm,0,80,0,100)

            self._encoder.reset()

            error = self._set_point - self._state

            self._control_effort += (error * self._kp) + (prev_error * self._kd) + (sum_error * self._ki)

            self._control_effort = min(max(0,self._control_effort),100)

            self._pwm.ChangeDutyCycle(self._control_effort)

            time.sleep(0.25)
            prev_error = error
            sum_error += error


    def set_speed(self,value):
        if (value >= 0):
            GPIO.output(self._fr_pin,GPIO.LOW)
            self._pwm.ChangeDutyCycle(value)
            self._set_point = value
        else:
            GPIO.output(self._fr_pin,GPIO.HIGH)
            self._pwm.ChangeDutyCycle(100 + value)
            self._set_point = 100 + value

    def remap(self, old_value, old_min, old_max, new_min, new_max):
        return (
            ((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)
        ) + new_min

LEFT_PWM = 2
LEFT_FR = 3
#LEFT_ENCODER = 18

RIGHT_PWM = 4
RIGHT_FR = 14
#RIGHT_ENCODER = 15

class MovementControl:

    def __init__(self):

        self._left_wheel = Motor(LEFT_PWM,LEFT_FR)
        self._right_wheel = Motor(RIGHT_PWM,RIGHT_FR)

    # should be a value between [-1,1]
    def set_speed(self,left_speed,right_speed):

        left_speed = min(max(-1,left_speed),1)
        right_speed= min(max(-1,right_speed),1)

        #convert to pwm
        left_speed = self.remap(left_speed,-1,1,-100,100)
        right_speed = self.remap(right_speed,-1,1,-100,100)

        print(left_speed)

        self._left_wheel.set_speed(left_speed)
        self._right_wheel.set_speed(right_speed)

    def remap(self, old_value, old_min, old_max, new_min, new_max):
        return (
            ((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)
        ) + new_min

class Robot:

    def __init__(self):
        self._movement_control = MovementControl()

        # power variables
        self._power = 0
        self._right_power = 0
        self._left_power = 0

    def apply_power(self, power):
        self._power = min(max(-1,power),1)

        self._movement_control.set_speed(self._power +self._left_power,self._power + self._right_power)

    def apply_steering_power(self, steering_power):
        steering_power = min(max(-0.4,steering_power),0.4)



        is_positive = self._power >= 0

        # turn right
        if (steering_power > 0):

            # right wheel may be too slow, if so, increase left instead
            if abs(self._power) - abs(steering_power) >= 0:
                print("Decrease right")

                # decrease right power
                self._right_power = -steering_power if is_positive else steering_power
            else:
                print("Increase left")

                #increase left power
                self._left_power = steering_power if is_positive else -steering_power

                print(self._left_power)

        # turn left
        elif (steering_power < 0):

            # left wheel may be too slow, if so, increase right instead
            if abs(self._power) - abs(steering_power) >= 0:

                #decrease left power
                self._left_power = steering_power if is_positive else -steering_power
            else:

                #increase right power
                self._right_power = -steering_power if is_positive else steering_power
        else:
            self._right_power = 0
            self._left_power = 0


        self._movement_control.set_speed(self._power + self._left_power, self._power + self._right_power)


#robot = Robot()
#robot.apply_power(1.0)
#
#while (True):
#    try:
#        time.sleep(10)
#    except KeyboardInterrupt:
#        robot.apply_power(0)
#        exit()


## if you dont cleanup, you might run into the issue of Runtime error:Failed to add edge detection
#GPIO.cleanup()

