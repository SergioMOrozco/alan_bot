#! /home/sorozco/computer_vision/bin/python3
import rospy
from std_msgs.msg import Float32
#from pynput.keyboard import Key, Listener
#from pynput import keyboard
from enum import Enum

class RobotMovement:

    ## max_power + max_turn needs to be <= 1.0
    def __init__(self, max_power = 0.6 , max_steering_power = 0.4):

        self.power = 0
        self.speed = 0.5
        self.steering_power = 0

        self.max_power = max_power
        self.max_steering_power = max_steering_power
        rospy.init_node('robot_movement')
        self.right_wheel_pub = rospy.Publisher('wheel_power_right'        ,Float32,queue_size=1)
        self.left_wheel_pub = rospy.Publisher('wheel_power_left'        ,Float32,queue_size=1)
        self.rate = rospy.Rate(10)

    def publish_left_wheel_command(self,speed):
        while not rospy.is_shutdown():
            connections = self.left_wheel_pub.get_num_connections()
            if connections > 0 :
                self.left_wheel_pub.publish(Float32(speed))
                break
            else:
                self.rate.sleep()

    def publish_right_wheel_command(self,speed):
        while not rospy.is_shutdown():
            connections = self.right_wheel_pub.get_num_connections()
            if connections > 0 :
                self.right_wheel_pub.publish(Float32(speed))
                break
            else:
                self.rate.sleep()

    def apply_power(self,power):
        # map power from [0,1] to [0,max_power]
        self.power = self.remap(power,0,1,0,self.max_power)
        self.publish_left_wheel_command(self.power)
        self.publish_right_wheel_command(self.power)

    def apply_steering_power(self,steering_power):
        # map steering power from [-1,1] to [0,1]
        self.steering_power = self.remap(abs(steering_power),0,1,0,self.max_steering_power)

        # steering power > 0 means that power is being applied to left wheel
        if (steering_power > 0) :
            self.publish_left_wheel_command(self.power + self.steering_power)

        # steering power < 0 means that power is being applied to right wheel
        elif (steering_power < 0):
            self.publish_right_wheel_command(self.power + self.steering_power)

        else:
            self.publish_right_wheel_command(self.power)
            self.publish_left_wheel_command(self.power)



    def move_forward(self):
        rospy.loginfo("Moving Forward!")
        self.publish_left_wheel_command(self.speed)
        self.publish_right_wheel_command(self.speed)

    def move_backward(self):
        rospy.loginfo("Moving Backward!")
        self.publish_left_wheel_command(-self.speed)
        self.publish_right_wheel_command(-self.speed)

    def move_left(self):
        rospy.loginfo("Moving Left!")
        self.publish_left_wheel_command(-self.speed)
        self.publish_right_wheel_command(self.speed)

    def move_right(self):
        rospy.loginfo("Moving Right!")
        self.publish_left_wheel_command(self.speed)
        self.publish_right_wheel_command(-self.speed)

    def stop(self):
        rospy.loginfo("Stopping")
        self.publish_left_wheel_command(0)
        self.publish_right_wheel_command(0)

    def remap(self,old_value,old_min,old_max,new_min,new_max):
        return (((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min

    def set_speed(self,speed):
        self.speed = speed
