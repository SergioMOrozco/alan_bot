from alan_core.motor import Motor
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

        # set to pwm 
        left_speed = min(max(-1,left_speed),1) * 100
        right_speed= min(max(-1,right_speed),1) * 100

        self._left_wheel.set_speed(left_speed)
        self._right_wheel.set_speed(right_speed)

