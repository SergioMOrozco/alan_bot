import os
import cv2
import tkinter as tk
import rospy
from tkinter import filedialog
from threading import Thread
from tkinter import ttk
from widgets.flask_stream import FlaskStream
from widgets.xbox_control_tab import XboxControlTab
from widgets.screen_control_tab import ScreenControlTab
from widgets.automated_control_tab import AutomatedControlTab
from std_msgs.msg import Float32, Empty

class MovementController(tk.LabelFrame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.LabelFrame.__init__(self,parent,*args,**kwargs)

        self.robot = robot
        self.parent = parent

        self._flask_stream = FlaskStream(self.robot)

        ## used for update left and right wheel labels
        self.right_wheel_sub = rospy.Subscriber('wheel_power_right',Float32,self.update_right_label)
        self.left_wheel_sub = rospy.Subscriber('wheel_power_left',Float32,self.update_left_label)
        self.stop_data_gather_sub = rospy.Subscriber('stop_data_gather',Empty,self.stop_gather_data)

        # display left hand controls
        self.left_wheel_label = tk.Label(self,text="Left Power: ")
        self.left_wheel_label.grid(row=0,column=0,padx=5)

        self.left_wheel_value_label = tk.Label(self,text="0.00")
        self.left_wheel_value_label.grid(row=0,column=1)

        self.stream_button = tk.Button(self,text="Start Stream", command=self.start_stream_button_press)
        self.stream_button.grid(row=0,column=2,padx=20)

        # display right hand controls
        self.right_wheel_label = tk.Label(self,text="Right Power: ")
        self.right_wheel_label.grid(row=0,column=3,padx=5)

        self.right_wheel_value_label = tk.Label(self,text="0.00")
        self.right_wheel_value_label.grid(row=0,column=4,padx=5)

        # create notebook with two movement control tabs
        self.notebook = tk.ttk.Notebook(self)
        self.screen_control_tab = ScreenControlTab(self.notebook,robot)
        self.xbox_control_tab = XboxControlTab(self.notebook,robot)
        self.automated_control_tab = AutomatedControlTab(self.notebook,robot)

        self.notebook.add(self.screen_control_tab,text="On Screen")
        self.notebook.add(self.xbox_control_tab,text="Xbox Control")
        self.notebook.add(self.automated_control_tab, text="Automation")

        self.notebook.grid(row=1,column=0,columnspan=5)

        # allow user to gather data
        self.data_gather_button = tk.Button(self,text="Gather Data", command=self.gather_data_button_press)
        self.data_gather_button.grid(row=2,column=2)

    def update_left_label(self,value):
        text = "%0.2f" % value.data
        self.left_wheel_value_label.configure(text=text)

    def update_right_label(self,value):
        text = "%0.2f" % value.data
        self.right_wheel_value_label.configure(text=text)

    def start_stream_button_press(self):
        self._flask_stream.start()
        self.stream_button.configure(text="Stop Stream",command=self.stop_stream_button_press)

    def stop_stream_button_press(self):
        self._flask_stream.stop()
        self.stream_button.configure(text="Start Stream",command=self.start_stream_button_press)


    def gather_data_button_press(self):

        data_path = filedialog.askdirectory(title='Create Folder')

        # ensure that user input a valid directory path before creating
        if data_path and not os.path.exists(data_path):

            os.makedirs(data_path)
            self.data_gather_button.configure(text="Stop Gather",command=self.stop_gather_data_button_press)

            ## start data gather thread
            Thread(target=self._gather_data, args=(data_path,)).start()


    def stop_gather_data(self,value):
        self.stop_gather_data_button_press()

    def stop_gather_data_button_press(self):
        self.data_path = ''
        self.data_gather_button.configure(text="Gather Data",command=self.gather_data_button_press)

        # stop gather thread
        self.is_gathering = False


    def _gather_data(self,data_path):

        ## preproccessing steps for data gather
        self.robot.start_video_stream()
        self.is_gathering = True  
        image_counter = 0
        label_file = open(data_path + '/labels.txt', 'w')

        while True:

            # break out of gather thread
            if not self.is_gathering:
                break

            # Capture frame-by-frame
            frame = self.robot.get_video_capture()
            steering_power = self.robot.get_steering_power()

            label_file.write(str(steering_power) + '\n')
            cv2.imwrite(data_path + '/' + f'{image_counter}' + '.jpg',frame)
            image_counter += 1
            
        label_file.close()
