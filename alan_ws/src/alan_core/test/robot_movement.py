#! /home/sorozco/computer_vision/bin/python3
from alan_core.robot_movement import RobotMovement
import rosunit
import unittest
import rostest
import rospy
import time
from std_msgs.msg import Float32


class TestRobotMovementForward(unittest.TestCase):

    def setUp(self):
        self.speed = .8
        self.rm = RobotMovement(self.speed)
        self.right_success = False
        self.left_success = False

    def right_callback(self,msg):
        self.right_success = round(msg.data,1) == round(self.speed,1)

    def left_callback(self,msg):
        self.left_success = round(msg.data,1) == round(self.speed,1)

    def runTest(self):

        ## mock receiver
        right_sub = rospy.Subscriber('wheel_power_right',Float32, self.right_callback)
        left_sub = rospy.Subscriber('wheel_power_left',Float32, self.left_callback)

        ## attempt to move forward
        self.rm.move_forward()

        timeout_t = time.time() + 10.0

        ## wait for 10 seconds
        while not rospy.is_shutdown() and not (self.right_success and self.left_success) and time.time() < timeout_t:
            time.sleep(0.1)

        ## assert if the message was ever received
        self.assert_(self.left_success and self.right_success)

class TestRobotMovementBackward(unittest.TestCase):

    def setUp(self):
        self.speed = .8
        self.rm = RobotMovement(self.speed)
        self.right_success = False 
        self.left_success = False 

    def right_callback(self,msg):
        self.right_success = round(msg.data,1) == round(-self.speed,1)

    def left_callback(self,msg):
        self.left_success = round(msg.data,1) == round(-self.speed,1)

    def runTest(self):

        ## mock receiver
        right_sub = rospy.Subscriber('wheel_power_right',Float32, self.right_callback)
        left_sub = rospy.Subscriber('wheel_power_left',Float32, self.left_callback)

        ## attempt to move backward
        self.rm.move_backward()

        timeout_t = time.time() + 10.0

        ## wait for 10 seconds
        while not rospy.is_shutdown() and not (self.right_success and self.left_success) and time.time() < timeout_t:
            time.sleep(0.1)

        ## assert if the message was ever received
        self.assert_(self.left_success and self.right_success)

class TestRobotMovementLeft(unittest.TestCase):

    def setUp(self):
        self.speed = .8
        self.rm = RobotMovement(self.speed)
        self.right_success = False 
        self.left_success = False 

    def right_callback(self,msg):
        self.right_success = round(msg.data,1) == round(self.speed,1)

    def left_callback(self,msg):
        self.left_success = round(msg.data,1) == round(-self.speed,1)

    def runTest(self):

        ## mock receiver
        right_sub = rospy.Subscriber('wheel_power_right',Float32, self.right_callback)
        left_sub = rospy.Subscriber('wheel_power_left',Float32, self.left_callback)

        ## attempt to move left
        self.rm.move_left()

        timeout_t = time.time() + 10.0

        ## wait for 10 seconds
        while not rospy.is_shutdown() and not (self.right_success and self.left_success) and time.time() < timeout_t:
            time.sleep(0.1)

        ## assert if the message was ever received
        self.assert_(self.left_success and self.right_success)

class TestRobotMovementRight(unittest.TestCase):

    def setUp(self):
        self.speed = .8
        self.rm = RobotMovement(self.speed)
        self.right_success = False 
        self.left_success = False 

    def right_callback(self,msg):
        self.right_success = round(msg.data,1) == round(-self.speed,1)

    def left_callback(self,msg):
        self.left_success = round(msg.data,1) == round(self.speed,1)

    def runTest(self):

        ## mock receiver
        right_sub = rospy.Subscriber('wheel_power_right',Float32, self.right_callback)
        left_sub = rospy.Subscriber('wheel_power_left',Float32, self.left_callback)

        ## attempt to move right 
        self.rm.move_right()

        timeout_t = time.time() + 10.0

        ## wait for 10 seconds
        while not rospy.is_shutdown() and not (self.right_success and self.left_success) and time.time() < timeout_t:
            time.sleep(0.1)

        ## assert if the message was ever received
        self.assert_(self.left_success and self.right_success)

class TestRobotMovementStop(unittest.TestCase):

    def setUp(self):
        self.speed = 0.8
        self.rm = RobotMovement(self.speed)
        self.right_success = False 
        self.left_success = False 

    def right_callback(self,msg):
        self.right_success = round(msg.data,1) == 0 

    def left_callback(self,msg):
        self.left_success = round(msg.data,1) == 0

    def runTest(self):

        ## mock receiver
        right_sub = rospy.Subscriber('wheel_power_right',Float32, self.right_callback)
        left_sub = rospy.Subscriber('wheel_power_left',Float32, self.left_callback)

        ## attempt to move forward
        self.rm.move_forward()
        time.sleep(1.0)
        self.rm.stop()

        timeout_t = time.time() + 10.0

        ## wait for 10 seconds
        while not rospy.is_shutdown() and not (self.right_success and self.left_success) and time.time() < timeout_t:
            time.sleep(0.1)

        ## assert if the message was ever received
        self.assert_(self.left_success and self.right_success)

class RobotMovementTestSuite(unittest.TestSuite):
    def __init__(self):
        super(RobotMovementTestSuite,self).__init__()
        self.addTest(TestRobotMovementForward())
        self.addTest(TestRobotMovementBackward())
        self.addTest(TestRobotMovementLeft())
        self.addTest(TestRobotMovementRight())
        self.addTest(TestRobotMovementStop())
