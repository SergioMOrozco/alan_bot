import tkinter as tk
from alan_core.xbox_control import XboxController
from alan_core.robot_movement import RobotMovement
from alan_core.widgets.controller_log import ControllerLog

class XboxControlTab(tk.Frame):
    def __init__(self,parent,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self.movement = RobotMovement()

        # create xbox controller 
        self.xbox_controller = XboxController(self.movement)

        # button to connect controller
        self.connect_button= tk.Button(self, text="Connect Controlller")
        self.connect_button.bind('<ButtonPress-1>',self.connect_controller)

        self.logging_frame = ControllerLog(self,text="Logging")

        self.connect_button.pack()
        self.logging_frame.pack()

    def connect_controller(self,event):
        self.xbox_controller.connect()



