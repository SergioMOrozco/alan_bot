from threading import Thread
from alan_core.encoder import Encoder
import time
import RPi.GPIO as GPIO

class Motor:
    def __init__(self,pwm_pin,fr_pin,encoder_pin = None):
        self._state = 0
        self._control_effort = 0
        self._set_point = 0

        GPIO.setmode(GPIO.BOARD)

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

        if isinstance(encoder_pin,int):
            print("feed back loop")
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

