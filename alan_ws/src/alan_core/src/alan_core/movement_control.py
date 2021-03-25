import rospy
from std_msgs.msg import Float32
from alan_core.motor import Motor

LEFT_PWM = 2
LEFT_FR = 3
#LEFT_ENCODER = 18

RIGHT_PWM = 4
RIGHT_FR = 14
#RIGHT_ENCODER = 15

class MovementControl:

    def __init__(self):

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

        self._left_wheel.set_speed(left_speed)
        self._right_wheel.set_speed(right_speed)

