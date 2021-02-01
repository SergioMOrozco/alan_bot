#! /home/sorozco/computer_vision/bin/python3
# Released by rdb under the Unlicense (unlicense.org)
# Modified by Sergio Orozco for ROS compatibility
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import os, struct, array
import time
from fcntl import ioctl
from alan_core.robot_movement import RobotMovement

class XboxController():

    # These constants were borrowed from linux/input.h
    axis_names = {
        0x00 : 'x',
        0x01 : 'y',
        0x02 : 'z',
        0x03 : 'rx',
        0x04 : 'ry',
        0x05 : 'rz',
        0x06 : 'trottle',
        0x07 : 'rudder',
        0x08 : 'wheel',
        0x09 : 'gas',
        0x0a : 'brake',
        0x10 : 'hat0x',
        0x11 : 'hat0y',
        0x12 : 'hat1x',
        0x13 : 'hat1y',
        0x14 : 'hat2x',
        0x15 : 'hat2y',
        0x16 : 'hat3x',
        0x17 : 'hat3y',
        0x18 : 'pressure',
        0x19 : 'distance',
        0x1a : 'tilt_x',
        0x1b : 'tilt_y',
        0x1c : 'tool_width',
        0x20 : 'volume',
        0x28 : 'misc',
    }
    
    button_names = {
        0x120 : 'trigger',
        0x121 : 'thumb',
        0x122 : 'thumb2',
        0x123 : 'top',
        0x124 : 'top2',
        0x125 : 'pinkie',
        0x126 : 'base',
        0x127 : 'base2',
        0x128 : 'base3',
        0x129 : 'base4',
        0x12a : 'base5',
        0x12b : 'base6',
        0x12f : 'dead',
        0x130 : 'a',
        0x131 : 'b',
        0x132 : 'c',
        0x133 : 'x',
        0x134 : 'y',
        0x135 : 'z',
        0x136 : 'tl',
        0x137 : 'tr',
        0x138 : 'tl2',
        0x139 : 'tr2',
        0x13a : 'select',
        0x13b : 'start',
        0x13c : 'mode',
        0x13d : 'thumbl',
        0x13e : 'thumbr',
    
        0x220 : 'dpad_up',
        0x221 : 'dpad_down',
        0x222 : 'dpad_left',
        0x223 : 'dpad_right',
    
        # XBox 360 controller uses these codes.
        0x2c0 : 'dpad_left',
        0x2c1 : 'dpad_right',
        0x2c2 : 'dpad_up',
        0x2c3 : 'dpad_down',
    }

    def __init__(self):

        self._power = 0
        self._steering_power = 0
        self.movement = RobotMovement()

        # name of axes and buttons
        self.axis_map = []
        self.button_map = []

        self.looping = True

        # We'll store the states here.
        self.axis_states = {}
        self.button_states = {}

        self.print_available_devices()
        self.open_device()
        self.print_device_name()
        self.initialize_states()
        self.loop()

    def print_available_devices(self):
        # Iterate over the joystick devices.
        print('Available devices:')
        
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('  /dev/input/%s' % (fn))


    def open_device(self):
        # Open the joystick device.
        fn = '/dev/input/js0'
        print('Opening %s...' % fn)

        opened = False

        while not opened:
            if os.path.exists(fn):
                self.jsdev = open(fn, 'rb')
                opened = True
            else:
                print('No device found. Try again? (y/n): ')
                try_again = input()

                if (try_again == 'n'):
                    print('Exiting...')
                    exit(-1)

                opened = False
                time.sleep(3)


    def print_device_name(self):
        # Get the device name.
        #buf = bytearray(63)
        buf = array.array('B', [0] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
        print('Device name: %s' % js_name)


    def initialize_states(self):
        # Get number of axes
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]
        
        # Get number of buttons
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]
        
        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP
        
        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0
        
        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP
        
        for btn in buf[:num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0
        
        print('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
        print('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))

    def loop(self):
        # Main event loop
        while self.looping:
            evbuf = self.jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)
        
                if type & 0x80:
                     print("(initial)", end="")
        
                if type & 0x01:
                    button = self.button_map[number]
                    if button:
                        self.button_states[button] = value
                        if value:
                            print("%s pressed" % (button))

                        else:
                            print("%s released" % (button))

                        self.process_button(button,value)
        
                if type & 0x02:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        print("%s: %.3f" % (axis, fvalue))

                        self.send_axis_data(axis)

    def process_button(self,button,value):
        if button == 'start':
            if value:
                print ('shutting down due to pressing START...')
                self.shutdown()

        # b will move the robot backwards
        if button == 'b':
            if value:
                self.movement.apply_power(-1.0)
            else:
                self.movement.apply_power(0)

    def send_axis_data(self, axis):
        if axis == 'gas':
            prev_power = self._power
            send_power_data = True

            # map ranges from [-1,1] to [0,1]
            self._power = self.remap(self.axis_states[axis],-1,1,0,1)

            # Unfortunately, Arduino's do not have support for ROS Actions; therefore,
            # if we send too many movement commands, the wheels may get out of sync since 
            # publishers and subscribers are not gauranteed to get all the messages
            case = lambda x,y: x < self._power <= y
            
            if self._power == 0:
                send_power_data = prev_power != 0
                self._power = 0
            elif case(0,0.25):
                send_power_data = prev_power != 0.25
                self._power = 0.25
            elif case(0.25,0.5):
                send_power_data = prev_power != 0.5
                self._power = 0.5
            elif case(0.5,0.75):
                send_power_data = prev_power != 0.75
                self._power = 0.75
            elif case(0.75,1.0):
                send_power_data = prev_power != 1.0
                self._power = 1.0

            if send_power_data:
                self.movement.apply_power(self._power)

        elif axis == 'x':
            prev_steering_power = self._steering_power
            send_data = True

            self._steering_power = self.axis_states[axis]

            # Unfortunately, Arduino's do not have support for ROS Actions; therefore,
            # if we send too many movement commands, the wheels may get out of sync since 
            # publishers and subscribers are not gauranteed to get all the messages
            case_pos = lambda x,y: x < self._steering_power <= y
            case_neg = lambda x,y: x <= self._steering_power < y

            if self._steering_power == 0:
                send_power_data = prev_steering_power != 0
                self._steering_power = 0
            elif case_pos(0,0.25):
                send_data = prev_steering_power != .25
                self._steering_power = .25
            elif case_pos(0.25,0.5):
                send_data = prev_steering_power != 0.5
                self._steering_power = 0.5
            elif case_pos(0.5,0.75):
                send_data = prev_steering_power != 0.75
                self._steering_power = 0.75
            elif case_pos(0.75,1.0):
                send_data = prev_steering_power != 1.0
                self._steering_power = 1.0
            elif case_neg(-0.25,0):
                send_data = prev_steering_power != -0.25
                self._steering_power = -0.25
            elif case_neg(-0.5,-0.25):
                send_data = prev_steering_power != -0.5
                self._steering_power = -0.5
            elif case_neg(-0.75,-0.5):
                send_data = prev_steering_power != -0.75
                self._steering_power = -0.75
            elif case_neg(-1.0,-0.75):
                send_data = prev_steering_power != -1.0
                self._steering_power = -1.0

            if send_data:
                self.movement.apply_steering_power(self._steering_power)


    def remap(self,old_value,old_min,old_max,new_min,new_max):
        return (((old_value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min

    def shutdown(self):
        self.looping = False


if __name__ == "__main__":
    control = XboxController()
