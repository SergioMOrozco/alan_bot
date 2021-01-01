#! /home/sorozco/computer_vision/bin/python3
from tkinter import *
from alan_core.robot_movement import RobotMovement
import time


class RobotControl():
    def __init__(self):
        self.movement = RobotMovement()

        time.sleep(3.0)

        self.create_and_display_widgets()

    def create_and_display_widgets(self):

        ## root widget (window)
        self.root = Tk()

        ## create and bind buttons
        self.forward_button= Button(self.root, text="W")
        self.forward_button.grid(row=0,column=1)
        self.forward_button.bind('<ButtonPress-1>',self.click_forward)
        self.forward_button.bind('<ButtonRelease-1>',self.button_released)

        self.left_button= Button(self.root, text="A")
        self.left_button.grid(row=1,column=0)
        self.left_button.bind('<ButtonPress-1>',self.click_left)
        self.left_button.bind('<ButtonRelease-1>',self.button_released)

        self.backward_button= Button(self.root, text="S")
        self.backward_button.grid(row=2,column=1)
        self.backward_button.bind('<ButtonPress-1>',self.click_backward)
        self.backward_button.bind('<ButtonRelease-1>',self.button_released)

        self.right_button= Button(self.root, text="D")
        self.right_button.grid(row=1,column=2)
        self.right_button.bind('<ButtonPress-1>',self.click_right)
        self.right_button.bind('<ButtonRelease-1>',self.button_released)

        ## start main loop. This actually displays the root window onto the screen
        self.root.mainloop()

    def click_forward(self,event):
        self.movement.move_forward()

    def click_left(self,event):
        self.movement.move_left()

    def click_backward(self,event):
        self.movement.move_backward()

    def click_right(self,event):
        self.movement.move_right()

    def button_released(self,event):
        self.movement.stop()

control = RobotControl()
