#!/usr/bin/env python

import cv2
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

try:
    import pubs
    import subs
    import video
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs
    import bluerov.video as video

from geometry_msgs.msg import TwistStamped, Pose
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import JointState, Joy

from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut

from mavros_msgs.srv import SetMode, SetModeRequest
from scipy.spatial.transform import Rotation as R



class Code(object):

    """Class to provide user access

    Attributes:
        cam (Video): Video object, get video stream
        pub (Pub): Pub object, do topics publication
        sub (Sub): Sub object, subscribe in topics
    """

    def __init__(self):
        super(Code, self).__init__()

        # Do what is necessary to start the process
        # and to leave gloriously
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.disarm()
        self.is_armed = False

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()

        self.pub.subscribe_topic('/mavros/rc/override', OverrideRCIn)
        self.pub.subscribe_topic('/mavros/setpoint_velocity/cmd_vel', TwistStamped)
        self.pub.subscribe_topic('/BlueRov2/body_command', JointState)

        self.sub.subscribe_topic('/joy', Joy)
        self.sub.subscribe_topic('/mavros/battery', BatteryState)
        self.sub.subscribe_topic('/mavros/rc/in', RCIn)
        self.sub.subscribe_topic('/mavros/rc/out', RCOut)
        self.sub.subscribe_topic('/imu_pose', Pose)

        self.cam = None
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("BlueRov2/image",Image)
        try:
            video_udp_port = rospy.get_param("/user_node/video_udp_port")
            rospy.loginfo("video_udp_port: {}".format(video_udp_port))
            self.cam = video.Video(video_udp_port)
        except Exception as error:
            rospy.loginfo(error)
            self.cam = video.Video()


    def arm(self):
        """ Arm the vehicle and trigger the disarm
        """
        self.arm_service(True)

        # Disarm is necessary when shutting down
        rospy.on_shutdown(self.disarm)

    def disarm(self):
            self.arm_service(False)


    @staticmethod
    def pwm_to_thrust(pwm):
        """Transform pwm to thruster value
        The equation come from:
            https://colab.research.google.com/notebook#fileId=1CEDW9ONTJ8Aik-HVsqck8Y_EcHYLg0zK

        Args:
            pwm (int): pwm value

        Returns:
            float: Thrust value
        """
        return -3.04338931856672e-13*pwm**5 \
            + 2.27813523978448e-9*pwm**4 \
            - 6.73710647138884e-6*pwm**3 \
            + 0.00983670053385902*pwm**2 \
            - 7.08023833982539*pwm \
            + 2003.55692021905


    def reset(self):
        rospy.loginfo('RESETING SQUARE ...')
        # set all values to zero (1500 in rc terms) (just to be safe)
        overridezero = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.pub.set_data('/mavros/rc/override', overridezero)
        # disarm
        self.disarm()
        self.is_armed = False

    def square_side(self, duration, quat_ini, rot):
        """Class to draw one side of the square

        Attributes:
            duration (float): duration of the forward movement
            quat_ini: initial angle
            rot (float): rotation to make
        """
        # First, straight front during specified time
        second_ini = rospy.get_time()
        while (rospy.get_time() < second_ini + duration):
            move_front = [1500, 1500, 1500, 1500, 1800, 1500, 1500, 1500]
            self.pub.set_data('/mavros/rc/override', move_front)

        # Then stop
        stop = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        self.pub.set_data('/mavros/rc/override', stop)

        # Then rotate 45 degrees
        ang_ini = R.from_quat(quat_ini)
        ang_ini_z = ang_ini.as_euler('zxy', degrees = True)[0]

        while (not rospy.on_shutdown(self.disarm)):
            current_orientation = self.sub.get_data()['/imu_pose']['orientation']
            current_quat = R.from_quat(current_orientation)
            current_z = current_quat.as_euler('zxy', degrees = True)[0]

            if(current_z < ang_ini_z + rot):
                rotate = [1500, 1500, 1500, 1500, 1500, 1200, 1500, 1500]
                self.pub.set_data('/mavros/rc/override', move_front)
            else:

                stop = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
                self.pub.set_data('/mavros/rc/override', stop)
                break







    def run(self):
        """Run user code
        """
        while not rospy.is_shutdown():
            time.sleep(0.1)

            # Get orientation data from imu topic
            orientation = self.sub.get_data()['imu_pose']['orientation']
            # Get joystick data
            joy = self.sub.get_data()['joy']['axes']
            joy_buttons = self.sub.get_data()['joy']['buttons']

            if joy_buttons[7] and not joy_buttons[6]:
                self.arm()
                self.is_armed = True

            elif joy_buttons[6]: # when pushing BACK button, reset square
                self.reset()


            try:
                if not self.cam.frame_available():
                    continue

                # Show video output
                frame = self.cam.frame()
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
            except Exception as error:
                print('imshow error:', error)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)



if __name__ == "__main__":
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Code()
    code.run()
