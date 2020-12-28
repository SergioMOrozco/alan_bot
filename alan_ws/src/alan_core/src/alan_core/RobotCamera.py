#! /home/sorozco/computer_vision/bin/python3
from __future__ import print_function
from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2

## parse the arguments on run
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--display", type=int,default=-1, help="Whether or not frames should be displayed")
args = vars(ap.parse_args())

## create a threaded video stream, allow the camera sensor to warmlup, and start the FPS counter
vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()

while(True):
    # Capture frame-by-frame, resize, and flip
    frame = vs.read()

    # flip to correct rotation
    frame = cv2.flip(frame,0)
    frame = cv2.flip(frame,1)

    if args['display'] > 0:
        # Display the resulting frame
        cv2.imshow('frame',frame)

        #show frame for one ms
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # update FPS counter
    fps.update()

fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))


# When everything done, release the capture
vs.stop()
cv2.destroyAllWindows()
