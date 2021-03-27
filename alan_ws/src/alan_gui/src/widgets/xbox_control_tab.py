import tkinter as tk
from alan_core.xbox_control import XboxController
from widgets.controller_log import ControllerLog

class XboxControlTab(tk.Frame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self._robot = robot

        # create xbox controller 
        self.xbox_controller = XboxController(self._robot)

        # button to connect controller
        self.connect_button= tk.Button(self, text="Connect Controlller")
        self.connect_button.bind('<ButtonPress-1>',self.connect_controller)

        self.logging_frame = ControllerLog(self,text="Logging")

        self.connect_button.pack()
        self.logging_frame.pack()

    def connect_controller(self,event):
        self.xbox_controller.connect()



