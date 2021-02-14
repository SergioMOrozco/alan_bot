import tkinter as tk
import cv2
import os
import rospy
from tkinter import filedialog
from alan_core.pi_video_stream import PiVideoStream
from PIL import Image,ImageTk
from std_msgs.msg import Float32

class StreamViewer(tk.LabelFrame):
    def __init__(self,parent,*args,**kwargs):

        self.right_wheel_sub = rospy.Subscriber('wheel_power_right',Float32,self.update_right_label)
        self.left_wheel_sub = rospy.Subscriber('wheel_power_left',Float32,self.update_left_label)

        tk.LabelFrame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self.is_streaming = False
        self.camera = PiVideoStream()

        self.data_path = ''

        # display left hand controls
        self.left_wheel_label = tk.Label(self,text="Left Power: ")
        self.left_wheel_label.grid(row=0,column=0)

        self.left_wheel_value_label = tk.Label(self,text="N/A")
        self.left_wheel_value_label.grid(row=0,column=1)

        # allow user to start or stop stream 
        self.stream_button= tk.Button(self, text="Start Stream", command=self.start_stream)
        self.stream_button.grid(row=0,column=2)

        # display right hand controls
        self.right_wheel_label = tk.Label(self,text="Right Power: ")
        self.right_wheel_label.grid(row=0,column=3)

        self.right_wheel_value_label = tk.Label(self,text="N/A")
        self.right_wheel_value_label.grid(row=0,column=4)

        self.frame = tk.Frame(self, highlightthickness=1)
        self.frame.grid(row=1,column=0,columnspan=5)

        self.label = tk.Label(self.frame,padx=200,pady=200)
        self.label.grid(row=1,column=0,columnspan=5)

        self.data_gather_button = tk.Button(self,text="Gather Data", command=self.gather_data)
        self.data_gather_button.grid(row=2,column=2)

    def update_left_label(self,value):
        text = "%0.2f" % value.data
        self.left_wheel_value_label.configure(text=text)

    def update_right_label(self,value):
        text = "%0.2f" % value.data
        self.right_wheel_value_label.configure(text=text)

    def start_stream(self):
        self.stream_button.configure(text="Stop Stream", command=self.stop_stream)

        # allow stream loop to begin
        self.is_streaming = True

        # start camera only if it is not already started
        if (self.camera.stopped):
            self.camera.start()

        # begin streaming
        self.stream()

    def stop_stream(self):
        self.stream_button.configure(text="Start Stream", command=self.start_stream)

        # stop stream looping
        self.is_streaming = False

        # stop camera
        self.camera.stop()

    def stream(self):
        try:

            # stop stream loop
            if not self.is_streaming:
                return

            # Capture frame-by-frame
            frame = self.camera.read()

            # flip to correct rotation
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)

            # display stream frame onto screen
            frame = Image.fromarray(frame)
            frame = ImageTk.PhotoImage(frame)
            self.label.config(image=frame)
            self.label.image =frame
            
            # call stream function again in 1ms
            self.label.after(1,lambda:self.stream())

        except:
            self.camera.stop()
            return

    def gather_data(self):
        data_path = filedialog.askdirectory(title='Create Folder')

        # ensure that user input a valid directory path before creating
        if data_path and not os.path.exists(data_path):
            os.makedirs(data_path)
            self.data_path = data_path
            self.data_gather_button.configure(text="Stop Gather",command=self.stop_gather)

            # code to start data gather

    def stop_gather(self):
            self.data_path = ''
            self.data_gather_button.configure(text="Gather Data",command=self.gather_data)

            # code to stop data gather

