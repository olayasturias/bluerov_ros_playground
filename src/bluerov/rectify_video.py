#!/usr/bin/env python
import rospy
import sys,os
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

class CorrectImg():
    def __init__(self, apply_mask):
        # Create rov pose publisher of odometry values
        self.apply_mask = apply_mask
        self.img_pub = rospy.Publisher('/BlueRov2/image_calibrated',
                                       Image,
                                       queue_size=10)
        cam_info_sub = rospy.Subscriber('/BlueRov2/camera_info',
                                        CameraInfo,
                                        self.info_callback)
        image_sub = rospy.Subscriber('/BlueRov2/image' ,
                                     Image,
                                     self.img_callback)

    def img_callback(self, data):
        rospy.logdebug('Received image topic...')
        bridge = CvBridge()
        # Read and convert data
        self.color_frame = bridge.imgmsg_to_cv2(data, "bgr8")
        undistorted = self.correct(self.color_frame)
        small = cv2.resize(undistorted,(0,0), fx=0.5, fy=0.5)
        cropped = self.crop(small)

        rospy.logdebug('Publishing corrected image...')
        image_message = bridge.cv2_to_imgmsg(cropped, encoding="bgr8")
        self.img_pub.publish(image_message)

        self.gaussblur(cv2.cvtColor(cropped,cv2.COLOR_BGR2HSV))


    def info_callback(self, data):
        self.K = np.reshape(data.K,(3,3))
        self.R = np.reshape(data.R,(3,3))
        self.D = data.D
        self.width = data.width
        self.height = data.height
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K,# camera matrix
                                                          self.D,# distorsion coef
                                                          (data.width,data.height),
                                                          0,# alpha
                                                          (data.width,data.height))


    def correct(self, img):
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.K,
                                                         self.D[-1],
                                                         self.R,
                                                         self.newcameramtx,
                                                         (self.width,self.height),
                                                         cv2.CV_16SC2)
        undistorted_img = cv2.remap(self.color_frame, map1, map2,
                                    interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)

        return undistorted_img

    def gaussblur(self, img):
        blur  = cv2.GaussianBlur(img,(11,11),11)

        # cv2.imshow('gauss',blur)
        cv2.imshow('blur',cv2.cvtColor(img,cv2.COLOR_HSV2BGR))


        div = cv2.divide(img,blur)

        floatblur = blur.astype(float)

        div2 = np.divide(img,floatblur)
        # for element in div2:
        #     for pix in element:
        #         for n in pix:
        #             if n == np.nan:
        #                 n = 0.0
        #
        # print div2.astype(int)
        norm = np.array([])
        norm = cv2.normalize(div,norm, alpha = 0.0, beta = 255.0, norm_type=cv2.NORM_MINMAX)
        # norm = np.linalg.norm(div2,2)

        # cv2.imshow('norm',norm)
        cv2.imshow('norm',cv2.cvtColor(norm,cv2.COLOR_HSV2BGR))

        # equ = cv2.equalizeHist(norm)
        #
        # cv2.imshow('equ', equ)

        cv2.waitKey(1)

        return blur


    def crop(self, img):
        mask = np.ones(img.shape[:2], np.uint8)*255
        h, w = img.shape[:2]
        print self.apply_mask

        if self.apply_mask:
            for j in range(int(2*h/3),h):
                for i in range(0,int(3 * (w/h) * (j-3*h/4))):
                    mask[j][i] = 0

            p = 0
            for j in range(2*h/3,h):
                for i in range(w-int(p*2.2),w):
                    mask[j][i] = 0
                p=p+1

            p = 0
            for j in range(0,5*h/12):
                for i in range(w-int(p*0.5)-13,w):
                    mask[j][i] = 0
                p=p+1

        masked_img = cv2.bitwise_and(img,img,mask = mask)

        return masked_img






if __name__ == '__main__':
    try:
        rospy.init_node('img_rectify', log_level=rospy.DEBUG)
        try:
            apply_mask = rospy.get_param('~mask')
        except:
            apply_mask = True
        rospy.logdebug('apply mask %s', apply_mask)
        rate = rospy.Rate(1) # 1 Hz
        ci = CorrectImg(apply_mask)
        rospy.sleep(1)
        cv2.destroyAllWindows()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
