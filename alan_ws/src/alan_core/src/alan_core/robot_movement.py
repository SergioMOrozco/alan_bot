#! /home/sorozco/computer_vision/bin/python3
import rospy
from std_msgs.msg import Float32
#from pynput.keyboard import Key, Listener
#from pynput import keyboard
from enum import Enum

class MovementType(Enum):
    Undefined = 0
    Forward = 1
    Backward = 2
    Left = 3
    Right = 4
    Stopped = 5

class RobotMovement:
    def __init__(self,speed = 0.01):
        rospy.init_node('robot_movement')
        self.speed = speed
        self.right_wheel_pub = rospy.Publisher('wheel_power_right'        ,Float32,queue_size=1)
        self.left_wheel_pub = rospy.Publisher('wheel_power_left'        ,Float32,queue_size=1)
        self._movement_type = MovementType.Undefined
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

    def move_forward(self):
        if (self._movement_type != MovementType.Forward):
            rospy.loginfo("Moving Forward!")
            self.publish_left_wheel_command(self.speed)
            self.publish_right_wheel_command(self.speed)
            self._movement_type = MovementType.Forward

    def move_backward(self):
        if (self._movement_type != MovementType.Backward):
            rospy.loginfo("Moving Backward!")
            self.publish_left_wheel_command(-self.speed)
            self.publish_right_wheel_command(-self.speed)
            self._movement_type = MovementType.Backward

    def move_left(self):
        if (self._movement_type != MovementType.Left):
            rospy.loginfo("Moving Left!")
            self.publish_left_wheel_command(-self.speed)
            self.publish_right_wheel_command(self.speed)
            self._movement_type = MovementType.Left

    def move_right(self):
        if (self._movement_type != MovementType.Right):
            rospy.loginfo("Moving Right!")
            self.publish_left_wheel_command(self.speed)
            self.publish_right_wheel_command(-self.speed)
            self._movement_type = MovementType.Right

    def stop(self):
        if (self._movement_type != MovementType.Stopped):
            rospy.loginfo("Stopping")
            self.publish_left_wheel_command(0)
            self.publish_right_wheel_command(0)
            self._movement_type = MovementType.Stopped

    def set_speed(self,speed):
        self.speed = speed

#def on_press(key, movement):
#    try:
#        if key.char == 'w':
#            movement.move_forward()
#        elif key.char == 'a':
#            movement.move_left()
#        elif key.char == 's':
#            movement.move_backward()
#        elif key.char == 'd':
#            movement.move_right()
#    except AttributeError:
#        pass
#
#def on_release(key, movement):
#    if key == keyboard.Key.esc:
#        # Stop listener
#        movement.stop()
#        return False
#
#if __name__ == "__main__":
#    rospy.init_node('robot_movement_node')
#    movement = RobotMovement()
#
#    on_press_func = lambda key: on_press(key, movement)
#    on_release_func = lambda key: on_release(key, movement)
#    # Collect events until released
#    with Listener(
#        on_press=on_press_func,
#        on_release=on_release_func) as listener:
#        listener.join()
#
