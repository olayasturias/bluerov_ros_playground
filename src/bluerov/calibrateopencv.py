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
    def __init__(self):
        self.images = []
        self.checkerboard = (7,5)

        subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW

    def fish_calib(self):
        _img_shape = None
        objp = np.zeros((1, self.checkerboard[0]*self.checkerboard[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2)
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        for img in self.images:
            if _img_shape == None:
                _img_shape = img.shape[:2]
            else:
                assert _img_shape == img.shape[:2], "All images must share the same size."

            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                imgpoints.append(corners)

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
                calibration_flags,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )
        print("Found " + str(N_OK) + " valid images for calibration")
        print("DIM=" + str(_img_shape[::-1]))
        print("K=np.array(" + str(K.tolist()) + ")")
        print("D=np.array(" + str(D.tolist()) + ")")

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while True:
            while True:
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)


class CalibrationNode:
    def __init__(self, boards, service_check = True, synchronizer = message_filters.TimeSynchronizer, flags = 0):
        if service_check:
            # assume any non-default service names have been set.  Wait for the service to become ready
            for svcname in ["camera", "left_camera", "right_camera"]:
                remapped = rospy.remap_name(svcname)
                if remapped != svcname:
                    fullservicename = "%s/set_camera_info" % remapped
                    print "Waiting for service", fullservicename, "..."
                    try:
                        rospy.wait_for_service(fullservicename, 5)
                        print "OK"
                    except rospy.ROSException:
                        print "Service not found"
                        rospy.signal_shutdown('Quit')

        self._boards = boards
        self._calib_flags = flags

        msub = message_filters.Subscriber('image', sensor_msgs.msg.Image)
        msub.registerCallback(self.queue_monocular)

        self.set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"),
                                                          sensor_msgs.srv.SetCameraInfo)

        self.q_mono = Queue.Queue()


        self.c = None

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

    def redraw_monocular(self, *args):
        pass

    def queue_monocular(self, msg):
        self.q_mono.put(msg)

    def handle_monocular(self, msg):
        if self.c == None:
            self.c = MonoCalibrator(self._boards, self._calib_flags)
            #print 'BBBBBBBBOOOOOOOOOOOOOOARRRRRRDS',self._boards.gridsize

        # This should just call the MonoCalibrator
        drawable = self.c.handle_msg(msg)
        #print drawable.scrib.shape[1]
        self.displaywidth = drawable.scrib.shape[1]
        print drawable
        print drawable.scrib
        self.redraw_monocular(drawable)

    def check_set_camera_info(self, response):
        if response.success:
            return True

        for i in range(10):
            print "!" * 80
        print
        print "Attempt to set camera info failed: " + response.status_message
        print
        for i in range(10):
            print "!" * 80
        print
        rospy.logerr('Unable to set camera info for calibration. Failure message: %s' % response.status_message)
        return False

    def do_upload(self):
        self.c.report()
        print self.c.ost()
        info = self.c.as_message()

        rv = True
        if self.c.is_mono:
            response = self.set_camera_info_service(info)
            rv = self.check_set_camera_info(response)
        else:
            response = self.set_left_camera_info_service(info[0])
            rv = rv and self.check_set_camera_info(response)
            response = self.set_right_camera_info_service(info[1])
            rv = rv and self.check_set_camera_info(response)
        return rv



class OpenCVCalibrationNode(CalibrationNode):
    """ Calibration node with an OpenCV Gui """

    def __init__(self, *args):

        CalibrationNode.__init__(self, *args)
        cv2.namedWindow("display")
        #self.font = cv2.cv.InitFont(cv2.CV_FONT_HERSHEY_SIMPLEX, 0.20, 1, thickness = 2)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        #self.button = cv.LoadImage("%s/button.jpg" % roslib.packages.get_pkg_dir(PKG))
        cv2.setMouseCallback("display", self.on_mouse)
        cv2.createTrackbar("scale", "display", 0, 100, self.on_scale)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN and self.displaywidth < x:
            if self.c.goodenough:
                if 180 <= y < 280:
                    self.c.do_calibration()
            if self.c.calibrated:
                if 280 <= y < 380:
                    self.c.do_save()
                elif 380 <= y < 480:
                    # Only shut down if we set camera info correctly, #3993
                    if self.do_upload():
                        rospy.signal_shutdown('Quit')



    def waitkey(self):
        k = cv.WaitKey(6)
        if k in [27, ord('q')]:
            rospy.signal_shutdown('Quit')
        return k

    def on_scale(self, scalevalue):
        if self.c.calibrated:
            self.c.set_alpha(scalevalue / 100.0)

    def button(self, dst, label, enable):
        #cv.Set(dst, (255, 255, 255))
        print dst
        size = dst.shape
        print size
        if enable:
            color = [155, 155, 80]
        else:
            color = [224, 224, 224]
        cv2.circle(dst, (size[0] / 2, size[1] / 2), min(size) / 2, color, -1)
        ((w, h), _) = cv.GetTextSize(label, self.font)
        cv.putText(dst, label, ((size[0] - w) / 2, (size[1] + h) / 2), self.font, (255,255,255))

    def buttons(self, display):
        x = self.displaywidth
        print x
        print display
        # self.button(cv.GetSubRect(display, (x,180,100,100)), "CALIBRATE", self.c.goodenough)
        self.button(display[180:280][x:x+100], "CALIBRATE", self.c.goodenough)
        # self.button(cv.GetSubRect(display, (x,280,100,100)), "SAVE", self.c.calibrated)
        self.button(display[280:380][x:x+100], "SAVE", self.c.calibrated)
        # self.button(cv.GetSubRect(display, (x,380,100,100)), "COMMIT", self.c.calibrated)
        self.button(display[380:480][x:x+100], "COMMIT", self.c.calibrated)

    def y(self, i):
        """Set up right-size images"""
        return 30 + 40 * i

    def screendump(self, im):
        i = 0
        while os.access("/tmp/dump%d.png" % i, os.R_OK):
            i += 1
        cv.SaveImage("/tmp/dump%d.png" % i, im)

    def redraw_monocular(self, drawable):
        height,width,channels = drawable.scrib.shape
        display = np.array((max(480, height), width + 100,channels), np.uint8)
        print display
        #display = cv.CreateMat(max(480, height), width + 100, cv.CV_8UC3)
        np.zeros(display)
        # np.copy(drawable.scrib, cv.GetSubRect(display, (0,0,width,height)))
        drawable.scrib*display[0:width][0:height]

        # cv.Set(cv.GetSubRect(display, (width,0,100,height)), (255, 255, 255))
        #cv2.set(display[width:0][100:height], (255, 255, 255))

        self.buttons(display)
        if not self.c.calibrated:
            if drawable.params:
                 for i, (label, lo, hi, progress) in enumerate(drawable.params):
                    (w,_),_ = cv.GetTextSize(label, self.font)
                    cv.PutText(display, label, (width + (100 - w) / 2, self.y(i)), self.font, (0,0,0))
                    color = (0,255,0)
                    if progress < 1.0:
                        color = (0, int(progress*255.), 255)
                    cv.Line(display,
                            (int(width + lo * 100), self.y(i) + 20),
                            (int(width + hi * 100), self.y(i) + 20),
                            color, 4)

        else:
            cv.PutText(display, "lin.", (width, self.y(0)), self.font, (0,0,0))
            linerror = drawable.linear_error
            if linerror < 0:
                msg = "?"
            else:
                msg = "%.2f" % linerror
                #print "linear", linerror
            cv.PutText(display, msg, (width, self.y(1)), self.font, (0,0,0))

        self.show(display)

    def show(self, im):
        cv.ShowImage("display", im)
        if self.waitkey() == ord('s'):
            self.screendump(im)


def main():
    from optparse import OptionParser, OptionGroup
    parser = OptionParser("%prog --size SIZE1 --square SQUARE1 [ --size SIZE2 --square SQUARE2 ]",
                          description=None)
    group = OptionGroup(parser, "Chessboard Options",
                        "You must specify one or more chessboards as pairs of --size and --square options.")
    #group.add_option("--pattern",
    #                 type="string", default="chessboard",
    #                 help="calibration pattern to detect - 'chessboard' or 'circles'")
    group.add_option("-s", "--size",
                     action="append", default=[],
                     help="chessboard size as NxM, counting interior corners (e.g. a standard chessboard is 7x7)")
    group.add_option("-q", "--square",
                     action="append", default=[],
                     help="chessboard square size in meters")
    parser.add_option_group(group)
    group = OptionGroup(parser, "ROS Communication Options")
    group.add_option("--approximate",
                     type="float", default=0.0,
                     help="allow specified slop (in seconds) when pairing images from unsynchronized stereo cameras")
    group.add_option("--no-service-check",
                     action="store_false", dest="service_check", default=True,
                     help="disable check for set_camera_info services at startup")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Calibration Optimizer Options")
    group.add_option("--fix-principal-point",
                     action="store_true", default=False,
                     help="fix the principal point at the image center")
    group.add_option("--fix-aspect-ratio",
                     action="store_true", default=False,
                     help="enforce focal lengths (fx, fy) are equal")
    group.add_option("--zero-tangent-dist",
                     action="store_true", default=False,
                     help="set tangential distortion coefficients (p1, p2) to zero")
    group.add_option("-k", "--k-coefficients",
                     type="int", default=2, metavar="NUM_COEFFS",
                     help="number of radial distortion coefficients to use (up to 6, default %default)")
    parser.add_option_group(group)
    group = OptionGroup(parser, "Deprecated Options")
    group.add_option("--rational-model",
                     action="store_true", default=False,
                     help="enable distortion coefficients k4, k5 and k6 (for high-distortion lenses)")
    group.add_option("--fix-k1", action="store_true", default=False,
                     help="do not change the corresponding radial distortion coefficient during the optimization")
    group.add_option("--fix-k2", action="store_true", default=False)
    group.add_option("--fix-k3", action="store_true", default=False)
    group.add_option("--fix-k4", action="store_true", default=False)
    group.add_option("--fix-k5", action="store_true", default=False)
    group.add_option("--fix-k6", action="store_true", default=False)
    parser.add_option_group(group)
    options, args = parser.parse_args()

    if len(options.size) != len(options.square):
        parser.error("Number of size and square inputs must be the same!")

    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    for (sz, sq) in zip(options.size, options.square):
        size = tuple([int(c) for c in sz.split('x')])
        boards.append(ChessboardInfo(size[0], size[1], float(sq)))

    if options.approximate == 0.0:
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateSynchronizer, options.approximate)

    num_ks = options.k_coefficients
    # Deprecated flags modify k_coefficients
    if options.rational_model:
        print "Option --rational-model is deprecated"
        num_ks = 6
    if options.fix_k6:
        print "Option --fix-k6 is deprecated"
        num_ks = min(num_ks, 5)
    if options.fix_k5:
        print "Option --fix-k5 is deprecated"
        num_ks = min(num_ks, 4)
    if options.fix_k4:
        print "Option --fix-k4 is deprecated"
        num_ks = min(num_ks, 3)
    if options.fix_k3:
        print "Option --fix-k3 is deprecated"
        num_ks = min(num_ks, 2)
    if options.fix_k2:
        print "Option --fix-k2 is deprecated"
        num_ks = min(num_ks, 1)
    if options.fix_k1:
        print "Option --fix-k1 is deprecated"
        num_ks = 0

    calib_flags = 0
    if options.fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if options.fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if options.zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if (num_ks > 3):
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if (num_ks < 6):
        calib_flags |= cv2.CALIB_FIX_K6
    if (num_ks < 5):
        calib_flags |= cv2.CALIB_FIX_K5
    if (num_ks < 4):
        calib_flags |= cv2.CALIB_FIX_K4
    if (num_ks < 3):
        calib_flags |= cv2.CALIB_FIX_K3
    if (num_ks < 2):
        calib_flags |= cv2.CALIB_FIX_K2
    if (num_ks < 1):
        calib_flags |= cv2.CALIB_FIX_K1

    rospy.init_node('cameracalibrator')
    node = OpenCVCalibrationNode(boards, options.service_check, sync, calib_flags)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except Exception, e:
        import traceback
        traceback.print_exc()
