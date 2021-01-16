#! /home/sorozco/computer_vision/bin/python3
from tkinter import *
from std_msgs.msg import Float32MultiArray
import rospy

class PidCalibrationControl():
    def __init__(self):
        rospy.init_node('pid_calibration')
        self.pid_calibration_pub = rospy.Publisher('pid_calibration',Float32MultiArray,queue_size=1)
        self.create_and_display_widgets()

    def create_and_display_widgets(self):

        ## root widget (window)
        self.root = Tk()

        self.kp_label = Label(self.root,text="Kp")
        self.kp_label.grid(row=0,column=0)

        self.kp_entry = Entry(self.root)
        self.kp_entry.grid(row=0,column=1)

        self.ki_label = Label(self.root,text="Ki")
        self.ki_label.grid(row=1,column=0)

        self.ki_entry = Entry(self.root)
        self.ki_entry.grid(row=1,column=1)

        self.kd_label = Label(self.root,text="Kd")
        self.kd_label.grid(row=2,column=0)

        self.kd_entry = Entry(self.root)
        self.kd_entry.grid(row=2,column=1)

        self.accept_button = Label(self.root,text="Accept")
        self.accept_button.grid(row=4,column=1)
        self.accept_button.bind('<Button-1>',self.accept_pid_settings)

        self.root.mainloop()

    def accept_pid_settings(self,event):
        kp = float(self.kp_entry.get())
        ki = float(self.ki_entry.get())
        kd = float(self.kd_entry.get())
        
        message = Float32MultiArray()
        message.data = [kp,ki,kd]
        self.pid_calibration_pub.publish(message)


control = PidCalibrationControl()
