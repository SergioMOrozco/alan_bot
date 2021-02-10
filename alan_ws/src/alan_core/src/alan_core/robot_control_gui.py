#! /home/sorozco/computer_vision/bin/python3
import tkinter as tk
#from tkinter import ttk
from alan_core.robot_movement import RobotMovement
from alan_core.pi_video_stream import PiVideoStream
from alan_core.xbox_control import XboxController
from alan_core.widgets.movement_controller import MovementController
from PIL import Image,ImageTk
import time
import cv2


class RobotControl(tk.Frame):
    def __init__(self,parent,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent

        self.movement_controller = MovementController(self,text="Movement")

        self.movement_controller.grid(row=0,column=0,rowspan=2)

        self.movement = RobotMovement()
        self.camera = PiVideoStream()
        self.xbox_controller = XboxController(self.movement)
 
        ## give robot time to boot up
        time.sleep(2.0)

        #self.create_and_display_widgets()

    def create_and_display_widgets(self):

        ## root widget (window)
        self.root = Tk()
        self.root.geometry("800x600")

        self.title_label = Label(self.root,text="Surge.IO")
        self.title_label.config(font=("Courier",44))


        # Left Hand Movement Frame
        self.movement_frame = LabelFrame(self.root,text="Movement")
        self.tab_control = ttk.Notebook(self.movement_frame)
        self.tab_screen = Frame(self.tab_control)
        self.tab_xbox = Frame(self.tab_control)
        self.tab_control.add(self.tab_screen,text="On Screen") 
        self.tab_control.add(self.tab_xbox,text="Xbox Control") 

        # screen tab
        self.forward_button= Button(self.tab_screen, text="W", anchor=N, padx=70, pady=70)
        self.left_button= Button(self.tab_screen, text="A", anchor=W, padx=30,pady=70)
        self.backward_button= Button(self.tab_screen, text="S", anchor=S,padx=70,pady=70)
        self.right_button= Button(self.tab_screen, text="D", anchor=E,padx=30,pady=70)

        # xbox tab
        self.controller_connect_button= Button(self.tab_xbox, text="Connect Controlller")

        # Right Hand Stream Frame
        self.stream_frame = LabelFrame(self.root,text="Stream")

        self.left_wheel_label = Label(self.stream_frame,text="Left Power: ")
        self.stream_button_text = StringVar()
        self.stream_button_text.set("Start Steam")
        self.stream_button= Button(self.stream_frame, textvariable=self.stream_button_text)
        self.right_wheel_label = Label(self.stream_frame,text="Right Power: ")


        # stream 
        self.frame = LabelFrame(self.stream_frame)
        self.label = Label(self.frame,padx=200,pady=200)

        self.movement_frame.grid(row=0,column=0,rowspan=2)
        self.stream_frame.grid(row=1,column=1)
        self.title_label.grid(row=0,column=1)


        self.tab_control.pack()

        ## create and bind buttons
        self.forward_button.grid(row=0,column=1)
        self.forward_button.bind('<ButtonPress-1>',self.click_forward)
        self.forward_button.bind('<ButtonRelease-1>',self.button_released)


        self.left_button.grid(row=1,column=0)
        self.left_button.bind('<ButtonPress-1>',self.click_left)
        self.left_button.bind('<ButtonRelease-1>',self.button_released)

        self.backward_button.grid(row=2,column=1)
        self.backward_button.bind('<ButtonPress-1>',self.click_backward)
        self.backward_button.bind('<ButtonRelease-1>',self.button_released)

        self.right_button.grid(row=1,column=2)
        self.right_button.bind('<ButtonPress-1>',self.click_right)
        self.right_button.bind('<ButtonRelease-1>',self.button_released)

        self.controller_connect_button.pack()
        self.controller_connect_button.bind('<ButtonPress-1>',self.connect_controller)

        self.left_wheel_label.grid(row=0,column=0)

        self.stream_button.grid(row=0,column=1)
        self.stream_button.bind('<Button-1>',self.start_stream)

        self.right_wheel_label.grid(row=0,column=2)

        self.frame.grid(row=1,column=0,columnspan=3)
        self.label.grid(row=1,column=0,columnspan=3)

        ## start main loop. This actually displays the root window onto the screen
        self.root.mainloop()

    def connect_controller(self,event):
        self.xbox_controller.connect()

    def click_forward(self,event):
        self.movement.apply_power(1.0)

    def click_left(self,event):
        self.movement.apply_steering_power(-1.0)

    def click_backward(self,event):
        self.movement.apply_power(-1.0)

    def click_right(self,event):
        self.movement.apply_steering_power(1.0)

    def button_released(self,event):
        self.movement.apply_power(0)

    def start_stream(self,event):
        self.stream_button_text.set("Stop Stream")
        self.stream_button.bind('<Button-1>',self.stop_stream)

        # allow stream loop to begin
        self.stream_stopped = False

        # start camera only if it is not already started
        if (self.camera.stopped):
            self.camera.start()

        # begin streaming
        self.stream()

    def stop_stream(self,event):
        self.stream_button_text.set("Start Stream")
        self.stream_button.bind('<Button-1>',self.start_stream)

        # stop stream looping
        self.stream_stopped = True

        # stop camera
        self.camera.stop()

    def stream(self):
        try:

            # stop stream loop
            if (self.stream_stopped):
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
if __name__ == "__main__":
    root = tk.Tk()
    RobotControl(root).pack(side="top",fill="both",expand=True)
    root.mainloop()

