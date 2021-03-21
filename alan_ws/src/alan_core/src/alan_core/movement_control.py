#! /home/sorozco/computer_vision/bin/python3
import rospy
from std_msgs.msg import Float32

# from pynput.keyboard import Key, Listener
# from pynput import keyboard
from enum import Enum


class MovementControl:
    def __init__(self, max_power=1.0, max_steering_power=0.4):

        # power variables
        self.power = 0
        self._right_power = 0
        self._left_power = 0
        self.max_power = max_power
        self.max_steering_power = max_steering_power

        # ros variables
        self.right_wheel_pub = rospy.Publisher(
            "wheel_power_right", Float32, queue_size=1
        )
        self.left_wheel_pub = rospy.Publisher("wheel_power_left", Float32, queue_size=1)
        self.rate = rospy.Rate(10)

    def publish_left_wheel_command(self):
        while not rospy.is_shutdown():
            connections = self.left_wheel_pub.get_num_connections()
            if connections > 0:
                self.left_wheel_pub.publish(Float32(self._left_power))
                break
            else:
                self.rate.sleep()

    def publish_right_wheel_command(self):
        while not rospy.is_shutdown():
            connections = self.right_wheel_pub.get_num_connections()
            if connections > 0:
                self.right_wheel_pub.publish(Float32(self._right_power))
                break
            else:
                self.rate.sleep()

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
