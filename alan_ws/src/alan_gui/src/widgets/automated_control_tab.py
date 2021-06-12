import pycuda.driver as cuda
#import pycuda.autoinit # For automatic creation and cleanup of CUDA context
import tensorrt as trt
import tkinter as tk
import os
import numpy as np
import time
import cv2
import time
from threading import Thread

class AutomatedControlTab(tk.Frame):
    def __init__(self,parent,robot,*args,**kwargs):
        tk.Frame.__init__(self,parent,*args,**kwargs)
        self.parent = parent
        self.robot = robot

        self.drive_button = tk.Button(self, text="Drive",command=self.start_drive)
        self.drive_button.pack()


        self.is_driving = False

    def my_input_fn(self):
        # Input for a single inference call, for a network that has two input tensors:
        Inp1 = np.random.normal(size=(8, 16, 16, 3)).astype(np.float32)
        inp2 = np.random.normal(size=(8, 16, 16, 3)).astype(np.float32)
        yield (inp1, inp2)

    def start_drive(self):
        self._is_driving = True
        self.drive_button.configure(text="Stop Drive", command=self.stop_drive)
        self.robot.start_video_stream()
        Thread(target=self.driving, args=()).start()

    def stop_drive(self):
        self._is_driving = False
        self.drive_button.configure(text="Drive",command=self.start_drive)
        self.robot.apply_power(0)
        self.robot.apply_steering_power(0)

    def driving(self):
        #self.initialize_trt()
        # grabbed here: https://stackoverflow.com/questions/60372729/get-logicerror-explicit-context-dependent-failed-invalid-device-context-no
        cuda.init()
        device = cuda.Device(0)  # enter your Gpu id here
        ctx = device.make_context()

        self.trt_logger = trt.Logger(trt.Logger.INFO)
        self.runtime = trt.Runtime(self.trt_logger)
        with open("/home/sorozco/engine.plan", "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        self.stream = cuda.Stream()

        self.host_in = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(0)), dtype=np.float32)
        self.host_out = cuda.pagelocked_empty(trt.volume(self.engine.get_binding_shape(1)), dtype=np.float32)
        self.devide_in = cuda.mem_alloc(self.host_in.nbytes)
        self.devide_out = cuda.mem_alloc(self.host_out.nbytes)

        while (True):
            self.robot.apply_power(1.0)
            if not self._is_driving:
                break

            frame = self.robot.get_video_capture()
            speed = self.classify_image(frame)
            print(speed[0])
            self.robot.apply_steering_power(speed[0],False)
        ctx.pop()  # very important

    def classify_image(self,image):
        clean = self.clean_image(image)

        # do i need this??
        #image = image.transpose(2,0,1)

        bindings = [int(self.devide_in), int(self.devide_out)]
        np.copyto(self.host_in, np.ravel(clean))
        cuda.memcpy_htod_async(self.devide_in, self.host_in, self.stream)
        self.context.execute_async(bindings=bindings, stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.host_out, self.devide_out, self.stream)
        self.stream.synchronize()
        return self.host_out

    def clean_image(self,image):
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        image = cv2.GaussianBlur(image, (3, 3), 0)
        scaled = cv2.resize(image, (200, 66))
        #region = self.region_of_interest(scaled)
        scaled_pixels = self.scale_pixels(scaled)

        return scaled_pixels

    def scale_pixels(self,image):
        normalized = self.normalize_pixels(image)
        centered = self.center_pixels(normalized)
        return centered

    def normalize_pixels(self,image):
        pixels = image.astype("float32")
        pixels /= 255.0
        return pixels

    def center_pixels(self,image):
        pixels = image - image.mean()
        return pixels
