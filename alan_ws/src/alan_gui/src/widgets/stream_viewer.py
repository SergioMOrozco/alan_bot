import tkinter as tk
import cv2
import os
import rospy
from tkinter import filedialog
from PIL import Image,ImageTk
from std_msgs.msg import Float32

class StreamViewer(tk.LabelFrame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.LabelFrame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self.robot = robot

        self.right_wheel_sub = rospy.Subscriber('wheel_power_right',Float32,self.update_right_label)
        self.left_wheel_sub = rospy.Subscriber('wheel_power_left',Float32,self.update_left_label)

        self.image_counter = 0

        self.is_streaming = False

        self.label_file = None

        self.data_path = ''

        # display left hand controls
        self.left_wheel_label = tk.Label(self,text="Left Power: ")
        self.left_wheel_label.grid(row=0,column=0)

        self.left_wheel_value_label = tk.Label(self,text="0.00")
        self.left_wheel_value_label.grid(row=0,column=1)

        # allow user to start or stop stream 
        self.stream_button= tk.Button(self, text="Start Stream", command=self.start_stream_button_press)
        self.stream_button.grid(row=0,column=2)

        # display right hand controls
        self.right_wheel_label = tk.Label(self,text="Right Power: ")
        self.right_wheel_label.grid(row=0,column=3)

        self.right_wheel_value_label = tk.Label(self,text="0.00")
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

    def start_stream_button_press(self):
        self.stream_button.configure(text="Stop Stream", command=self.stop_stream_button_press)

        # can't allow user to gather data while streaming
        self.data_gather_button.configure(state=tk.DISABLED)

        self.start_stream(True)


    def stop_stream_button_press(self):
        self.stream_button.configure(text="Start Stream", command=self.start_stream_button_press)

        # allow user the gather data once streaming is done
        self.data_gather_button.configure(state=tk.NORMAL)

        self.stop_stream()

    def start_stream(self,display = False, gather_data = False):
        # allow stream loop to begin
        self.is_streaming = True

        self.robot.start_video_stream()

        if gather_data:
            self.label_file = open(self.data_path + '/labels.txt', 'w')

        # begin streaming
        self.stream(display,gather_data)

    def stop_stream(self):
        # stop stream looping
        self.is_streaming = False

        if (not (self.label_file is None)):
            self.label_file.close()

        # reset image counter for next data gather
        self.image_counter = 0

    def stream(self, display, gather_data):
        try:
            # stop stream loop
            if not self.is_streaming:
                return

            # Capture frame-by-frame
            frame = self.robot.get_video_capture()

            # display stream frame onto screen
            if display:
                frame = cv2.resize(frame,(640,480))
                frame = Image.fromarray(frame)
                frame = ImageTk.PhotoImage(frame)
                self.label.config(image=frame)
                self.label.image =frame

            if gather_data:
                left_speed,right_speed = self.robot.get_power()
                self.label_file.write(str(left_speed) + ',' + str(right_speed) + '\n')
                cv2.imwrite(self.data_path + '/' + f'{self.image_counter}' + '.jpg',frame)
                self.image_counter += 1
            
            # call stream function again in 1ms
            self.label.after(1,lambda:self.stream(display,gather_data))

        except:
            self.robot.stop_video_stream()

            if (not (self.label_file is None)):
                self.label_file.close()
            return

    def gather_data(self):
        data_path = filedialog.askdirectory(title='Create Folder')

        # ensure that user input a valid directory path before creating
        if data_path and not os.path.exists(data_path):
            os.makedirs(data_path)
            self.data_path = data_path
            self.data_gather_button.configure(text="Stop Gather",command=self.stop_gather)

            # can't allow user to stream while gathering data
            self.stream_button.configure(state=tk.DISABLED)

            # start data gather
            self.start_stream(False,True)

    def stop_gather(self):
            self.data_path = ''
            self.data_gather_button.configure(text="Gather Data",command=self.gather_data)

            # allow user to stream when data gathering is finished
            self.stream_button.configure(state=tk.NORMAL)

            # stop data gather
            self.stop_stream()

