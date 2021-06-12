import cv2
from alan_core.movement_control import MovementControl
from alan_core.camera_control import CameraControl

class Robot:
    def __init__(self):
        self._movement_control = MovementControl()
        self._camera = CameraControl()

        # power variables
        self._power = 0
        self._right_steer_power = 0
        self._left_steer_power = 0

    def apply_power(self, power):
        self._power = min(max(-0.3,power),0.3)

        self._movement_control.set_speed(self._power +self._left_steer_power,self._power + self._right_steer_power)

    def apply_steering_power(self, steering_power, remap = True):

        if (remap):
            self._steering_power = self.remap(steering_power,-1.0,1.0,-0.2,0.2)
        else:
            self._steering_power = steering_power

        is_positive = self._power >= 0

        # turn right
        if (self._steering_power > 0):

            # right wheel may be too slow, if so, increase left instead
            if abs(self._power) - abs(self._steering_power) >= 0:

                # decrease right power
                self._right_steer_power = -self._steering_power if is_positive else self._steering_power
            else:

                #increase left power
                self._left_steer_power = self._steering_power if is_positive else -self._steering_power

        # turn left
        elif (self._steering_power < 0):

            # left wheel may be too slow, if so, increase right instead
            if abs(self._power) - abs(self._steering_power) >= 0:

                #decrease left power
                self._left_steer_power = self._steering_power if is_positive else -self._steering_power
            else:

                #increase right power
                self._right_steer_power = -self._steering_power if is_positive else self._steering_power
        else:

            # stop turning
            self._right_steer_power = 0
            self._left_steer_power = 0

        self._movement_control.set_speed(self._power + self._left_steer_power, self._power + self._right_steer_power)

    def get_power(self):
        return self._power 
    def get_steering_power(self):
        return self._steering_power

    def remap(self, old_value, old_min, old_max, new_min, new_max):
        return (
            ((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)
        ) + new_min

    def start_video_stream(self):

        # start camera only if it is not already started
        if (self._camera.stopped):
            self._camera.start()

    def stop_video_stream(self):
        self._camera.stop()

    def get_video_capture(self):

        frame = self._camera.read()

        return frame
