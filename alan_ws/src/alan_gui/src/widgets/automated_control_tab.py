import tkinter as tk
import numpy as np
import time
from cv2 import cv2
from tflite_runtime.interpreter import Interpreter
from threading import Thread

class AutomatedControlTab(tk.Frame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent
        self.robot = robot

        self.interpreter = Interpreter("/home/sorozco/dev/alan_bot/models/model.tflite")

        # allocates memory for the input and ouput tensors
        self.interpreter.allocate_tensors()

        self.drive_button = tk.Button(self, text="Drive",command=self.start_drive)
        self.drive_button.pack()

        self.is_driving = False

    def start_drive(self):
        self._is_driving = True
        self.drive_button.configure(text="Stop Drive", command=self.stop_drive)
        self.robot.start_video_stream()
        Thread(target=self.driving, args=()).start()

    def stop_drive(self):
        self._is_driving = False
        self.drive_button.configure(text="Drive",command=self.start_drive)

    def driving(self):

        while (True):
            if not self._is_driving:
                break

            frame = self.robot.get_video_capture()
            speed = self.classify_image(frame)
            self.robot._movement_control.set_speed(speed[0],speed[1])
            print(speed)
            #time.sleep(0.1)

    # reference https://blog.paperspace.com/tensorflow-lite-raspberry-pi/
    def classify_image(self,image):
            clean = self.clean_image(image)

            # get input tensor
            tensor_index = self.interpreter.get_input_details()[0]['index']
            input_tensor = self.interpreter.get_tensor(tensor_index)

            # set tensor equal to image
            input_tensor[:,:] = clean
            self.interpreter.set_tensor(tensor_index,input_tensor)

            #invoke the self.interpreter. Essentially this runs the model.
            self.interpreter.invoke()
            output_details = self.interpreter.get_output_details()[0]
            output = np.squeeze(self.interpreter.get_tensor(output_details['index']))

            return output

    def clean_image(self,image):

        scaled = self.scale_image(image)
        region = self.region_of_interest(scaled)

        return region

    def scale_image(self,image, scale=60):

        if scale == 100:
            return image

        # calculate the 50 percent of original dimensions
        width = int(image.shape[1] * scale / 100)
        height = int(image.shape[0] * scale / 100)

        # resize image
        return cv2.resize(image, (width, height))

    def region_of_interest(self,img):

        height, width, channels = img.shape

        vertices = np.array(
            [
                [
                    (0, height),
                    (0, height * 3 / 4),
                    (width / 5, height / 3),
                    (width * 4 / 5, height / 3),
                    (width, height * 3 / 4),
                    (width, height),
                ]
            ],
            np.int32,
        )

        # create empty mas
        mask = np.zeros_like(img)

        # mask only works for binary images
        match_mask_color = (255, 255, 255)

        # filly polygon with white given vertices
        cv2.fillPoly(mask, vertices, match_mask_color)

        # only take pixels where both the mask and image are white
        return cv2.bitwise_and(img, mask)

        
            


