import cv2
import time
import threading
from flask import Response, Flask

## Create the Flask object for the application
app = Flask("Flask Stream")
app.debug = False
app.use_reloader = False

global video_frame
video_frame = None

# Use locks for thread-safe viewing of frames in multiple browsers
global thread_lock
thread_lock = threading.Lock()

class FlaskStream():
    def __init__(self,robot):

        self.robot = robot

    def start(self):
        self.robot.start_video_stream()

        # Create a thread and attach the method that captures the image frames, to it
        process_thread = threading.Thread(target=self.captureFrames)
        process_thread.daemon = True

        # Start the thread
        process_thread.start()

        flask_thread = threading.Thread(target=self.flask_thread)
        flask_thread.daemon = True

        # Start the thread
        flask_thread.start()

        #self.flask_thread()

    def flask_thread(self):
        # start the Flask Web Application
        # While it can be run on any feasible IP, IP = 0.0.0.0 renders the web app on
        # the host machine's localhost and is discoverable by other machines on the same network 
        app.run("0.0.0.0", port="8000")

    def captureFrames(self):
        global video_frame, thread_lock
        while True :
            with thread_lock:
                video_frame = self.robot.get_video_capture()

            cv2.waitKey(1)

def encodeFrame():
    global thread_lock
    while True:
        with thread_lock:
            global video_frame
            return_key, encoded_image = cv2.imencode(".jpg", video_frame)

        # Output image as a byte array
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encoded_image) + b'\r\n')

@app.route("/")
def streamFrames():
    return Response(encodeFrame(), mimetype = "multipart/x-mixed-replace; boundary=frame")

