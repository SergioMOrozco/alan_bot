#! /home/sorozco/computer_vision/bin python3

from alan_core.pi_video_stream import PiVideoStream
import rosunit
import unittest
import sys
import time

class TestPiCameraFrame(unittest.TestCase):

    def setUp(self):
        self.pvs = PiVideoStream()

    ## test if frames are being read properly by PiVideoStream
    def runTest(self):
        self.pvs.start()

        ## return numpy array
        frame = self.pvs.read()

        self.pvs.stop()

        self.assertIsNotNone(frame,None)

class TestPiCameraStop(unittest.TestCase):

    def setUp(self):
        self.pvs = PiVideoStream()

    ## test if camera is properly stopping
    def runTest(self):
        self.pvs.start()
        self.pvs.stop()

        self.assertEqual(self.pvs.camera.closed,True)

class PiCameraTestSuite(unittest.TestSuite):

    def __init__(self):
        super(PiCameraTestSuite, self).__init__()
        self.addTest(TestPiCameraFrame())
        self.addTest(TestPiCameraStop())
