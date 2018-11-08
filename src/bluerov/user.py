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

from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import JointState, Joy

from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut

from mavros_msgs.srv import SetMode, SetModeRequest


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
        self.arm()
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
        rospy.wait_for_service('/mavros/cmd/arming')

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm_service(True)

        # Disarm is necessary when shutting down
        rospy.on_shutdown(self.disarm)


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


    def run(self):
        """Run user code
        """
        while not rospy.is_shutdown():
            time.sleep(0.1)
            # Try to get data
            try:
                rospy.loginfo(self.sub.get_data()['mavros']['battery']['voltage'])
                rospy.loginfo(self.sub.get_data()['mavros']['rc']['in']['channels'])
                rospy.loginfo(self.sub.get_data()['mavros']['rc']['out']['channels'])
            except Exception as error:
                print('Get data error:', error)

            try:
                # Get joystick data
                joy = self.sub.get_data()['joy']['axes']
                joy_buttons = self.sub.get_data()['joy']['buttons']

                if joy_buttons[7] and not joy_buttons[6]:
                    self.arm()
                    self.is_armed = True

                elif joy_buttons[6]:
                    # set all values to zero (1500 in rc terms) (just to be safe)
                    overridezero = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
                    self.pub.set_data('/mavros/rc/override', overridezero)
                    # disarm
                    self.disarm()
                    self.is_armed = False

                elif joy_buttons[0]:
                    rospy.wait_for_service('/mavros/set_mode')
                    nav_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    if self.is_armed:
                        resp_nav = nav_mode(SetModeRequest.MAV_MODE_MANUAL_ARMED,'')
                    else:
                        resp_nav = nav_mode(SetModeRequest.MAV_MODE_MANUAL_DISARMED,'')

                elif joy_buttons[1]:
                    rospy.wait_for_service('/mavros/set_mode')
                    nav_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    if self.is_armed:
                        resp_nav = nav_mode(SetModeRequest.MAV_MODE_STABILIZE_ARMED,'')
                    else:
                        resp_nav = nav_mode(SetModeRequest.MAV_MODE_STABILIZE_DISARMED,'')



                # rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
                override = [int(val*400 + 1500) for val in joy]

                for _ in range(len(override), 8):
                    override.append(0)
                override[5] = override[0]

                print override
                # Send joystick data as rc output into rc override topic
                # (fake radio controller)
                self.pub.set_data('/mavros/rc/override', override)
            except Exception as error:
                print('joy error:', error)

            try:
                # Get pwm output and send it to Gazebo model
                rc = self.sub.get_data()['mavros']['rc']['out']['channels']
                joint = JointState()
                joint.name = ["thr{}".format(u + 1) for u in range(5)]
                joint.position = [self.pwm_to_thrust(pwm) for pwm in rc]

                self.pub.set_data('/BlueRov2/body_command', joint)
            except Exception as error:
                print('rc error:', error)

            try:
                if not self.cam.frame_available():
                    continue

                # Show video output
                frame = self.cam.frame()
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
		except CvBridgeError as e:
			print(e)

            except Exception as error:
                print('imshow error:', error)

    def disarm(self):
        self.arm_service(False)


if __name__ == "__main__":
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Code()
    code.run()
