#! /usr/bin/python3
import tkinter as tk
from widgets.movement_controller import MovementController
from alan_core.robot import Robot
import time
import rospy


class RobotControl(tk.Frame):
    def __init__(self,parent,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self._robot = Robot()

        # Title of application
        self.title_label = tk.Label(self,text="Surge.IO")
        self.title_label.config(font=("Courier",44))
        #self.title_label.grid(row=0,column=1)
        self.title_label.pack()

        # Notebook which allows the user to control the robot in multiple ways
        self.movement_controller = MovementController(self,self._robot,text="Movement")
        #self.movement_controller.grid(row=0,column=0,rowspan=2)
        self.movement_controller.pack()

        # Give robot time to boot up
        time.sleep(2.0)

if __name__ == "__main__":
    rospy.init_node("robot_control_gui")
    root = tk.Tk()
    root.geometry("800x600")
    RobotControl(root).pack(side="top",fill="both",expand=True)
    root.mainloop()

