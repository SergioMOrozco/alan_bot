import rospy
import RPi.GPIO as GPIO

from std_msgs.msg import Float32
from alan_core.motor import Motor

ENABLE_PIN = 16
RIGHT_PWM = 33
RIGHT_FR = 35

LEFT_PWM = 32
LEFT_FR = 18

OFFSET = 2

class MovementControl:

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        # enable motor driver
        GPIO.setup(ENABLE_PIN,GPIO.OUT)
        GPIO.output(ENABLE_PIN,GPIO.HIGH)

        # ros variables
        self.right_wheel_pub = rospy.Publisher('wheel_power_right'        ,Float32,queue_size=10)
        self.left_wheel_pub = rospy.Publisher('wheel_power_left'        ,Float32,queue_size=10)

        self._left_wheel = Motor(LEFT_PWM,LEFT_FR)
        self._right_wheel = Motor(RIGHT_PWM,RIGHT_FR)

    # should be a value between [-1,1]
    def set_speed(self,left_speed,right_speed):
        # range from [-1,1]
        left_speed = min(max(-1,left_speed),1)
        right_speed= min(max(-1,right_speed),1)

        self.left_wheel_pub.publish(Float32(left_speed))
        self.right_wheel_pub.publish(Float32(right_speed))

        #set to pwm
        left_speed *= 100
        right_speed *= 100

        if (left_speed < 0 ):
            left_speed = max(min(left_speed + OFFSET,0) -100)
        else:
            left_speed = min(max(left_speed - OFFSET,0), 100)

        self._left_wheel.set_speed(left_speed)
        self._right_wheel.set_speed(right_speed)

