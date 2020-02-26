#!/usr/bin/env python

# ROS module
import rospy
# Mavlink ROS messages
from mavros_msgs.msg import Mavlink
from sensor_msgs.msg import FluidPressure
# pack and unpack functions to deal with the bytearray
from struct import pack, unpack
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import Imu
from pyquaternion import Quaternion



class ROVPose(object):
    def __init__(self, atm_pressure, water_density):
        rospy.loginfo('holi')
        rospy.Subscriber("/mavlink/from", Mavlink, self.bar30callback, queue_size = 1)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        self.prespub = rospy.Publisher('/bar30/pressure',FluidPressure,queue_size = 1)
        self.pose_sub = rospy.Subscriber('/imu_pose', Pose,
                                        self.posecallback, queue_size = 1)
        self.abs_pressure = 101325.0 # absolute pressure in Pa
        self.atm_pressure = atm_pressure # Pa
        self.water_density = water_density # kg/m3

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

            self.abs_pressure = press_abs*100 # convert hPa to Pa
            self.prespub.publish(fp)

    def imu_callback(self, data):
        # Read imu Data
        r = Quaternion(data.orientation.w, data.orientation.x,
                       data.orientation.y, data.orientation.z)
        # Define 180 degs rotation around y
        qy = Quaternion(axis=[0, 1, 0], angle=3.14159265)
        # Define 90 deg rotation around z
        qz = Quaternion(axis=[0, 0, 1], angle=3.14159265/2)
        # total rotations (order is important)
        qtot = qy * qz
        ry = qtot.rotate(r) # rotate r 
        ry = ry.normalised



        # Broadcast to tf
        tf_pub = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.rotation.x = ry[1]
        t.transform.rotation.y = ry[2]
        t.transform.rotation.z = ry[3]
        t.transform.rotation.w = ry[0]

        pressure_to_meters = (self.abs_pressure - self.atm_pressure)
        pressure_to_meters = pressure_to_meters / (self.water_density * 9.85)
        t.transform.translation.z = pressure_to_meters

        tf_pub.sendTransform(t)

    def posecallback(self, data):
        tf_pub = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation = data.position
        t.transform.rotation = data.orientation

        pressure_to_meters = (self.abs_pressure - self.atm_pressure)
        pressure_to_meters = pressure_to_meters / (self.water_density * 9.85)
        t.transform.translation.z = pressure_to_meters

        tf_pub.sendTransform(t)



if __name__ == '__main__':
    rospy.init_node('bar30_listener', anonymous=True,log_level=rospy.DEBUG)
    atm_pressure = rospy.get_param('~atm_pressure')
    rospy.logdebug("ATM PRESSURE %s", atm_pressure)
    water_density = rospy.get_param('~water_density')
    rospy.logdebug("WATER DENSITY %s", water_density)


    ROVPose(atm_pressure, water_density)

    rospy.spin()
