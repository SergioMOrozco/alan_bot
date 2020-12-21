## pi camera allows access to the RAspberry Pi camera module
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2

class PiVideoStream:
    def __init__(self,resolution=(320,240),framerate=32):
        print("TESTING")
        ##initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,format = "bgr",use_video_port=True)

        ## initialize the frame and variable use to indicate if thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
        #start the thread to read frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread stops
        for f in self.stream:
            #grab the frame from the stream and clear the stream in prep for next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indication variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return
    def read(self):
        # return the frame most recently read
        return self.frame
    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

