from threading import Thread
import cv2
import time


class CameraControl:
    def __init__(self, resolution=(1024,720), framerate=60):
        self.stopped = True

        self._gstreamer_pipeline = (("nvarguscamerasrc ! " +
        "video/x-raw(memory:NVMM), " +
        "width={0}, height={1}, " +
        "format=(string)NV12, framerate={2}/1 ! " +
        "nvvidconv flip-method={3} ! " +
        "video/x-raw, width={4}, height={5}, format=(string)BGRx ! " +
        "videoconvert ! " +
        "video/x-raw, format=(string)BGR ! appsink")
        .format(
            resolution[0],
            resolution[1],
            framerate,
            0,
            resolution[0],
            resolution[1],
        ))

    def start(self):
        print("CAMERA STARTED")
        self.stopped = False
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()

        ## give camera some time to warmup
        time.sleep(2.0)
        return self

    def update(self):
        print(self._gstreamer_pipeline,cv2.CAP_GSTREAMER)
        cap = cv2.VideoCapture(self._gstreamer_pipeline,cv2.CAP_GSTREAMER)

        # keep looping infinitely until the thread stops
        while True:
            ret,self.frame = cap.read()

            # if the thread indication variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                cap.release()
                return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

        ## give camera some time to cool down and deallocate resources
        time.sleep(2.0)
