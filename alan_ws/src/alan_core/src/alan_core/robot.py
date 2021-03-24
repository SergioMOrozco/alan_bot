#! /home/sorozco/computer_vision/bin/python3
from alan_core.movement_control import MovementControl

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

        # max steering power is 0.4
        steering_power = min(max(-0.4,steering_power),0.4)

        is_positive = self._power >= 0

        # turn right
        if (steering_power > 0):

            # right wheel may be too slow, if so, increase left instead
            if abs(self._power) - abs(steering_power) >= 0:

                # decrease right power
                self._right_power = -steering_power if is_positive else steering_power
            else:

                #increase left power
                self._left_power = steering_power if is_positive else -steering_power

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

            # stop turning
            self._right_power = 0
            self._left_power = 0

        self._movement_control.set_speed(self._power + self._left_power, self._power + self._right_power)
