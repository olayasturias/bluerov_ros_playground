#!/usr/bin/env python

# ROS module
import rospy
# Mavlink ROS messages
from mavros_msgs.msg import Mavlink
from sensor_msgs.msg import FluidPressure
# pack and unpack functions to deal with the bytearray
from struct import pack, unpack

import pandas as pd

class Bar30(object):
    def __init__(self):
        rospy.Subscriber("/mavlink/from", Mavlink, self.bar30callback, queue_size = 1)
        self.pub = rospy.Publisher('/bar30/pressure',FluidPressure,queue_size = 1)

    # Topic callback
    def bar30callback(self, data):
        # Check if message id is valid (I'm using SCALED_PRESSURE
        # and not SCALED_PRESSURE2)
        if data.msgid == 137:
            # Transform the payload in a python string
            p = pack("QQ", *data.payload64)
            # Transform the string in valid values
            # https://docs.python.org/2/library/struct.html
            time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)

            fp = FluidPressure()
            fp.header = data.header
            fp.fluid_pressure = press_abs
            fp.variance = 2.794

            self.pub.publish(fp)


def listener():
    rospy.init_node('bar30_listener', anonymous=True)

    Bar30()

    rospy.spin()

if __name__ == '__main__':
    listener()
