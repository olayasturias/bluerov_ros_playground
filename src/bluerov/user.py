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
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut, ManualControl

from mavros_msgs.srv import SetMode, SetModeRequest

from joysticks import joystick, axis


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
        self.pub.subscribe_topic('/mavros/manual_control/send', ManualControl)

        self.sub.subscribe_topic('/joy', Joy)
        self.sub.subscribe_topic('/mavros/battery', BatteryState)
        self.sub.subscribe_topic('/mavros/rc/in', RCIn)
        self.sub.subscribe_topic('/mavros/rc/out', RCOut)
        self.cam = None
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("BlueRov2/image",Image, queue_size = 5)
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


    def run(self):
        """Run user code
        """
        while not rospy.is_shutdown():
            time.sleep(0.1)
            # Try to get data
            # try:
            #     rospy.loginfo(self.sub.get_data()['mavros']['battery']['voltage'])
            #     rospy.loginfo(self.sub.get_data()['mavros']['rc']['in']['channels'])
            #     rospy.loginfo(self.sub.get_data()['mavros']['rc']['out']['channels'])
            # except Exception as error:
            #     print('Get data error:', error)

            try:
                # Get joystick data
                joy = self.sub.get_data()['joy']['axes']
                joy_buttons = self.sub.get_data()['joy']['buttons']

                # rc run between 1100 and 2000, a joy command is between -1.0 and 1.0
                rcread = [int(val*(-400) + 1500) for val in joy] # give RC value to joy
                override = [1500,1500,1500,1500,1500,1500,1500,1500]

                if joy_buttons[joystick['FSi6']['buttons']['SWA_down']] and not joy_buttons[joystick['FSi6']['buttons']['SWA_up']]: # Arm
                    self.arm()
                    self.is_armed = True

                elif joy_buttons[joystick['FSi6']['buttons']['SWA_up']] and not joy_buttons[joystick['FSi6']['buttons']['SWA_down']]: # Disarm
                    # set all values to zero (1500 in rc terms) (just to be safe)
                    overridezero = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
                    self.pub.set_data('/mavros/rc/override', overridezero)
                    # disarm
                    self.disarm()
                    self.is_armed = False

                if joy_buttons[joystick['FSi6']['buttons']['SWB_down']]:
                    # SWB down selects the GAIN mode for the VRA axis.
                    # That is, with the SWB down, moving the VRA will change the
                    # gain (increase or decrease) accordingly.

                    # Define manual control msg
                    manual_control_msg = ManualControl()
                    # https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
                    # Warning: Because of some legacy workaround, z will work between [0-1000]
                    # where 0 is full reverse, 500 is no output and 1000 is full throttle.
                    # x,y and r will be between [-1000 and 1000].
                    # joy command is between -1.0 and 1.0

                    # set all unnecessary parameters to a neutral value
                    manual_control_msg.x = 0
                    manual_control_msg.y = 0
                    manual_control_msg.z = 500
                    manual_control_msg.r = 0

                    if joy[joystick['FSi6']['axis']['VRA']] < 0: # Gain decrease

                        rospy.loginfo('GAIN DECREASE')

                        manual_control_msg.buttons = 1 << 9 #11th button
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)
                        manual_control_msg.buttons = 0
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)

                    if joy[joystick['FSi6']['axis']['VRA']] > 0: # gain increase
                        rospy.loginfo('GAIN INCREASE')
                        manual_control_msg.buttons = 1 << 10
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)
                        manual_control_msg.buttons = 0
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)

                if joy_buttons[joystick['FSi6']['buttons']['SWB_up']]:
                    # SWB down selects the LIGHT mode for the VRA axis.
                    # That is, with the SWB down, moving the VRA will change the
                    # light (brighter or dimmer) accordingly.

                    # Define manual control msg
                    manual_control_msg = ManualControl()
                    # https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
                    # Warning: Because of some legacy workaround, z will work between [0-1000]
                    # where 0 is full reverse, 500 is no output and 1000 is full throttle.
                    # x,y and r will be between [-1000 and 1000].
                    # joy command is between -1.0 and 1.0

                    # set all unnecessary parameters to a neutral value
                    manual_control_msg.x = 0
                    manual_control_msg.y = 0
                    manual_control_msg.z = 500
                    manual_control_msg.r = 0
                    if joy[joystick['FSi6']['axis']['VRA']] < 0: # lights manual_control_msg
                        rospy.loginfo('Lights dimmer')

                        manual_control_msg.buttons = 1 << 13
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)
                        manual_control_msg.buttons = 0
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)

                    elif joy[joystick['FSi6']['axis']['VRA']] > 0: # lights brighter
                        rospy.loginfo('Lights brighter')
                        manual_control_msg.buttons = 1 << 14
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)
                        manual_control_msg.buttons = 0
                        self.pub.set_data('/mavros/manual_control/send', manual_control_msg)


                if joy_buttons[joystick['FSi6']['buttons']['SWC_up']]:
                    rospy.wait_for_service('/mavros/set_mode')
                    nav_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    if self.is_armed:
                        resp_nav = nav_mode(custom_mode='MANUAL')
                    else:
                        resp_nav = nav_mode(custom_mode='MANUAL')

                elif joy_buttons[joystick['FSi6']['buttons']['SWC_center']]:
                    rospy.wait_for_service('/mavros/set_mode')
                    nav_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    if self.is_armed:
                        resp_nav = nav_mode(custom_mode='STABILIZE')
                    else:
                        resp_nav = nav_mode(custom_mode='STABILIZE')

                elif joy_buttons[joystick['FSi6']['buttons']['SWC_down']]:
                    rospy.wait_for_service('/mavros/set_mode')
                    nav_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                    if self.is_armed:
                        resp_nav = nav_mode(custom_mode='ALT_HOLD')
                    else:
                        resp_nav = nav_mode(custom_mode='ALT_HOLD')



                # assign values
                override[axis['updown']] = rcread[joystick['FSi6']['axis']['VL']]
                override[axis['roty']]   = rcread[joystick['FSi6']['axis']['HL']]
                override[axis['rotz']]   = rcread[joystick['FSi6']['axis']['HR']]
                override[axis['for-backwards']] =  rcread[joystick['FSi6']['axis']['VR']]

                override[axis['unknown']]= rcread[joystick['FSi6']['axis']['VRA']]
                override[axis['cam']]    = rcread[joystick['FSi6']['axis']['VRB']]
                override[axis['cam']+1]  = rcread[joystick['FSi6']['axis']['VRB']]




                for _ in range(len(override), 8):
                    override.append(0)

                # Send joystick data as rc output into rc override topic
                override[2] = 1500 # otherwise it starts spinning like crazy
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
                # cv2.imshow('frame', frame)
                # cv2.waitKey(1)
            except Exception as error:
                print('imshow error:', error)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            except CvBridgeError as e:
                print(e)





if __name__ == "__main__":
    try:
        rospy.init_node('user_node', log_level=rospy.INFO)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    code = Code()
    code.run()
