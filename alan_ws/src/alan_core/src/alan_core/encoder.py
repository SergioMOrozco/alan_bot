import RPi.GPIO as GPIO

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
