#! /home/sorozco/computer_vision/bin/python3
import rospy
from std_msgs.msg import Float32
from pynput.keyboard import Key, Listener

SPEED = 1
class RobotMovement:
    def __init__(self):
        self.right_wheel_pub = rospy.Publisher('wheel_power_left'        ,Float32,queue_size=1)
        self.left_wheel_pub = rospy.Publisher('wheel_power_right'        ,Float32,queue_size=1)

    def move_forward(self):
        self.right_wheel_pub.publish(Float32(SPEED))
        self.left_wheel_pub.publish(Float32(SPEED))

    def move_backward(self):
        self.right_wheel_pub.publish(Float32(-SPEED))
        self.left_wheel_pub.publish(Float32(-SPEED))

    def move_left(self):
        self.right_wheel_pub.publish(Float32(SPEED))
        self.left_wheel_pub.publish(Float32(-SPEED))

    def move_right(self):
        self.right_wheel_pub.publish(Float32(-SPEED))
        self.left_wheel_pub.publish(Float32(SPEED))

    def stop(self):
        self.right_wheel_pub.publish(Float32(0))
        self.left_wheel_pub.publish(Float32(0))

def on_press(key):
    print (key + " Pressed")
def on_release(key):
    print (key + " Pressed")


if __name__ == "__main__":
    rospy.init_node('robot_movement_node')
    movement = RobotMovement()

    # Collect events until released
    with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        listener.join()

