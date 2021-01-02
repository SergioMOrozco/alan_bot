#! /home/sorozco/computer_vision/bin/python3
from tkinter import *
from alan_core.robot_movement import RobotMovement
from alan_core.pi_video_stream import PiVideoStream
from PIL import Image,ImageTk
import time
import cv2


class RobotControl():
    def __init__(self):
        self.movement = RobotMovement()
        self.camera = PiVideoStream()

        ## give robot time to boot up
        time.sleep(2.0)

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

        self.stream_button_text = StringVar()
        self.stream_button_text.set("Start Steam")
        self.stream_button= Button(self.root, textvariable=self.stream_button_text)
        self.stream_button.grid(row=3,column=3)
        self.stream_button.bind('<Button-1>',self.start_stream)

        self.frame = Frame()
        self.label = Label(self.frame)
        self.frame.grid(row=4,column=3)
        self.label.grid(row=4,column=3)

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

    def start_stream(self,event):
        self.stream_button_text.set("Stop Stream")
        self.stream_button.bind('<Button-1>',self.stop_stream)
        self.stream()

    def stop_stream(self,event):
        self.stream_button_text.set("Start Stream")
        self.stream_button.bind('<Button-1>',self.start_stream)
        self.stream_stopped = True

    def stream(self):
        try:
            if (self.camera.stopped):
                self.stream_stopped = False
                self.camera.start()

            if (self.stream_stopped):
                # When everything done, release the capture
                self.camera.stop()
                return

            # Capture frame-by-frame
            frame = self.camera.read()

            # flip to correct rotation
            frame = cv2.flip(frame,0)
            frame = cv2.flip(frame,1)

            frame = Image.fromarray(frame)
            frame = ImageTk.PhotoImage(frame)
            self.label.config(image=frame)
            self.label.image =frame

            self.label.after(1,lambda:self.stream())

        except:
            self.camera.stop()
            return


        
            
    
control = RobotControl()
