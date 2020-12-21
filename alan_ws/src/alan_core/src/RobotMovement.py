#! /home/sorozco/computer_vision/bin/python3
import rospy
from std_msgs.msg import Float32
from pynput.keyboard import Key, Listener
from pynput import keyboard
from enum import Enum

SPEED = 1

class MovementType(Enum):
    Forward = 1
    Backward = 2
    Left = 3
    Right = 4
    Stopped = 5

class RobotMovement:
    def __init__(self):
        self.right_wheel_pub = rospy.Publisher('wheel_power_left'        ,Float32,queue_size=1)
        self.left_wheel_pub = rospy.Publisher('wheel_power_right'        ,Float32,queue_size=1)
        self._movement_type = MovementType.Stopped

    def move_forward(self):
        if (self._movement_type != MovementType.Forward):
            rospy.loginfo("Moving Forward!")
            self.right_wheel_pub.publish(Float32(SPEED))
            self.left_wheel_pub.publish(Float32(SPEED))
            self._movement_type = MovementType.Forward

    def move_backward(self):
        if (self._movement_type != MovementType.Backward):
            rospy.loginfo("Moving Backward!")
            self.right_wheel_pub.publish(Float32(-SPEED))
            self.left_wheel_pub.publish(Float32(-SPEED))
            self._movement_type = MovementType.Backward

    def move_left(self):
        if (self._movement_type != MovementType.Left):
            rospy.loginfo("Moving Left!")
            self.right_wheel_pub.publish(Float32(SPEED))
            self.left_wheel_pub.publish(Float32(-SPEED))
            self._movement_type = MovementType.Left

    def move_right(self):
        if (self._movement_type != MovementType.Right):
            rospy.loginfo("Moving Right!")
            self.right_wheel_pub.publish(Float32(-SPEED))
            self.left_wheel_pub.publish(Float32(SPEED))
            self._movement_type = MovementType.Right

    def stop(self):
        if (not self._movement_type == MovementType.Stopped):
            self.right_wheel_pub.publish(Float32(0))
            self.left_wheel_pub.publish(Float32(0))
            self._movement_type = MovementType.Stopped

def on_press(key, movement):
    try:
        if key.char == 'w':
            movement.move_forward()
        elif key.char == 'a':
            movement.move_left()
        elif key.char == 's':
            movement.move_backward()
        elif key.char == 'd':
            movement.move_right()
    except AttributeError:
        pass

def on_release(key, movement):
    if key == keyboard.Key.esc:
        # Stop listener
        movement.stop()
        return False

if __name__ == "__main__":
    rospy.init_node('robot_movement_node')
    movement = RobotMovement()

    on_press_func = lambda key: on_press(key, movement)
    on_release_func = lambda key: on_release(key, movement)
    # Collect events until released
    with Listener(
        on_press=on_press_func,
        on_release=on_release_func) as listener:
        listener.join()

