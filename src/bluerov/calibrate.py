#!/usr/bin/env python
PKG = 'camera_calibration'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys,os
from sensor_msgs.msg import Image
from camera_calibration.calibrator import MonoCalibrator,ChessboardInfo
import sensor_msgs.msg
import sensor_msgs.srv
import message_filters
#from camera_calibration.approxsync import ApproximateSynchronizer
import numpy as np

import os
import Queue
import threading
import functools
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'

import glob


class FisheyeCalibrator():
    def __init__(self,dir,dim):
        self.images = glob.glob(dir)
        self.checkerboard = dim

        self.subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        self.calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_FIX_SKEW

    def fish_calib(self):
        _img_shape = None
        objp = np.zeros((1, self.checkerboard[0]*self.checkerboard[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        for fname in self.images:
            rospy.logdebug('Reading calibration image.....')
            img = cv2.imread(fname)
            if _img_shape == None:
                _img_shape = img.shape[:2]
            else:
                assert _img_shape == img.shape[:2], "All images must share the same size."

            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                rospy.logdebug('Found corners')
                objpoints.append(objp)
                cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),self.subpix_criteria)
                imgpoints.append(corners)
        rospy.logdebug('Done with image reading')

        N_OK = len(objpoints)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        rms, _, _, _, _ = \
            cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                gray.shape[::-1],
                K,
                D,
                rvecs,
                tvecs,
                self.calibration_flags,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )
        print("Found " + str(N_OK) + " valid images for calibration")
        print("DIM=" + str(_img_shape[::-1]))
        print("K=np.array(" + str(K.tolist()) + ")")
        print("D=np.array(" + str(D.tolist()) + ")")



def main():

    rospy.init_node('cameracalibrator',log_level=rospy.DEBUG)
    dir = '$(find bluerov_ros_playground)/calibrationdata/*.png'
    dir = '/home/olaya/catkin_ws/src/bluerov_ros_playground/calibrationdata/*.png'
    dim = (7,5)
    node = FisheyeCalibrator(dir,dim)
    node.fish_calib()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        import traceback
        traceback.print_exc()
