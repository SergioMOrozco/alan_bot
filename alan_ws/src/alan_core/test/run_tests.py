import rosunit
import pi_camera_test_suite

# rosunit
rosunit.unitrun('alan_core', 'pi_camera_test_suite',
                'pi_camera_test_suite.PiCameraTestSuite')
