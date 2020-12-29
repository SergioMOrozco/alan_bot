#! /home/sorozco/computer_vision/bin/python3
import rosunit
import rostest
import pi_video_stream
import robot_movement

# rosunit
rosunit.unitrun('alan_core', 'pi_video_stream',
                'pi_video_stream.PiCameraTestSuite')
rostest.rosrun('alan_core', 'robot_movement',
                'robot_movement.RobotMovementTestSuite')
