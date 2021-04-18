import cv2
import time
import threading
from flask import Response, Flask

class FlaskStream():
    app = None

    def __init__(self,robot):

        self.robot = robot

        self.app = Flask("Flask Stream")
        self.app.debug = False
        self.app.use_reloader = False

        # Use locks for thread-safe viewing of frames in multiple browsers
        self.thread_lock = threading.Lock()

        self.video_frame = None

        self._stopped = True

        self.app.add_url_rule('/','index',self.streamFrames)

        # Start flask app
        self.flask_thread = threading.Thread(target=self.flask_thread)
        self.flask_thread.daemon = True

        self.flask_thread.start()

    def start(self):

        if (self._stopped):
            self._stopped = False

            self.robot.start_video_stream()

            # Create a thread and attach the method that captures the image frames, to it
            self.process_thread = threading.Thread(target=self.captureFrames)
            self.process_thread.daemon = True

            # Start the thread
            self.process_thread.start()

    def stop(self):
        self._stopped = True


    def flask_thread(self):

        # start the Flask Web Application
        # While it can be run on any feasible IP, IP = 0.0.0.0 renders the web app on
        # the host machine's localhost and is discoverable by other machines on the same network 
        self.app.run("0.0.0.0", port="8000")

    def captureFrames(self):
        while True :

            ## break out of thread on stop
            if (self._stopped):
                break

            with self.thread_lock:
                self.video_frame = self.robot.get_video_capture()

            cv2.waitKey(1)

    def displayFrames(self):
        while True:

            ## break out of thread on stop
            if (self._stopped):
                break

            with self.thread_lock:
                return_key, encoded = cv2.imencode(".jpg", self.video_frame)

            # Output image as a byte array
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
                bytearray(encoded) + b'\r\n')

    def streamFrames(self):
        return Response(self.displayFrames(), mimetype = "multipart/x-mixed-replace; boundary=frame")

