#!/usr/bin/env python
import rospy
import sys,os
import lowpass
import cv2
assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import threading
import Queue
import time


class ThreadWorker(threading.Thread):
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

class CorrectImg():
    def __init__(self, apply_mask):
        # Create rov pose publisher of odometry values
        self.apply_mask = apply_mask
        self.y_array = []
        self.img_pub = rospy.Publisher('/BlueRov2/image_calibrated',
                                       Image,
                                       queue_size=10)
        cam_info_sub = rospy.Subscriber('/BlueRov2/camera_info',
                                        CameraInfo,
                                        self.info_callback)
        image_sub = rospy.Subscriber('/BlueRov2/image' ,
                                     Image,
                                     self.img_callback)

        self.img_queue          = Queue.Queue(maxsize = 10)
        self.y_queue            = Queue.Queue(maxsize = 20)
        self.homomorphic_queue  = Queue.Queue(maxsize = 5)
        self.equalization_queue = Queue.Queue(maxsize = 5)
        self.gauss_queue        = Queue.Queue(maxsize = 5)
        self.lowpass_queue      = Queue.Queue(maxsize = 5)

        # low_pass_thread = ThreadWorker(self.img_queue, self.lowpasspixel)
        # low_pass_thread.setDaemon(True)
        # low_pass_thread.start()

        homo_thread = ThreadWorker(self.y_queue, self.homomorphic_filter)
        homo_thread.setDaemon(True)
        homo_thread.start()

        # equalization_thread = ThreadWorker(self.img_queue, self.yuv_equalization)
        # equalization_thread.setDaemon(True)
        # equalization_thread.start()
        #
        # gauss_thread = ThreadWorker(self.img_queue, self.gaussblur)
        # gauss_thread.setDaemon(True)
        # gauss_thread.start()



    def img_callback(self, data):
        rospy.logdebug('Received image topic...')
        bridge = CvBridge()
        # Read and convert data
        self.color_frame = bridge.imgmsg_to_cv2(data, "bgr8")
        undistorted = self.correct(self.color_frame)
        small = cv2.resize(undistorted,(0,0), fx=0.5, fy=0.5)
        cropped = self.crop(small)

        self.img_queue.put(cropped)

        img32  = np.float32(cropped)
        img_norm = img32/255

        yuv  = cv2.cvtColor(img_norm,cv2.COLOR_BGR2YUV)

        y = yuv[:, :, 0]
        u = yuv[:, :, 1]
        v = yuv[:, :, 2]
        self.y_queue.put(y)


        # rospy.logdebug('Publishing corrected image...')
        # image_message = bridge.cv2_to_imgmsg(equ_image, encoding="bgr8")
        # self.img_pub.publish(image_message)


        # crop32 = np.float32(cropped)/255
        #res = np.hstack((np.float32(cropped)/255, homo_image, blur))
        #cv2.imshow('original - homomorphic filtering - adaptive histogram equalization - blur', res)
        #res2 = np.hstack((np.float32(equ_image)/255, lp_image))
        cv2.imshow('crop', cropped)
        if not self.homomorphic_queue.empty():
            y = self.homomorphic_queue.get()
            # merged = cv2.merge((y,u,v))
            # homo = cv2.cvtColor(merged,cv2.COLOR_YUV2BGR)
            cv2.imshow('homo', y)
            cv2.moveWindow('homo', 40,30)
        if not self.equalization_queue.empty():
            cv2.imshow('equalization', self.equalization_queue.get())
            cv2.moveWindow('equalization', 552,30)
        if not self.lowpass_queue.empty():
            cv2.imshow('lowpass', self.lowpass_queue.get())
            cv2.moveWindow('lowpass', 40,414)
        if not self.gauss_queue.empty():
            cv2.imshow('gauss', self.gauss_queue.get())
            cv2.moveWindow('gauss', 552,414)


        cv2.waitKey(1)

    def printyuv(self,y):
        print y.shape
        return


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

        img  = np.float32(img)
        img = img/255

        img  = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)

        y = img[:, :, 0]
        u = img[:, :, 1]
        v = img[:, :, 2]

        blur  = cv2.GaussianBlur(y,(11,11),11)

        div = cv2.divide(y,blur)

        merged = cv2.merge((div,u,v))
        color = cv2.cvtColor(merged,cv2.COLOR_YUV2BGR)

        self.gauss_queue.put(color)

    def queue_get_all(self,q):
        items = []
        maxItemsToRetreive = 10
        for numOfItemsRetrieved in range(0, maxItemsToRetreive):
            try:
                if numOfItemsRetrieved == maxItemsToRetreive:
                    break
                items.append(q.get_nowait())
            except:
                break
        return items

    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def lowpasspixel(self, img):
        img  = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)
        rospy.logwarn(' Entering low pass filter function ')
        yp = img[:, :, 0]
        u = img[:, :, 1]
        v = img[:, :, 2]

        if not self.y_queue.empty() and self.y_queue.qsize>2:

            y_array = self.queue_get_all(self.y_queue)
            n,h,w = np.asarray(y_array).shape
            nelements = h*w

            #y_array = np.reshape(y_array,(n,1,nelements))
            longarray = []
            for image in y_array:
                longarray.append(image.ravel())
            longarray = np.asarray(longarray)

            print longarray.shape

        # print self.y_queue.qsize()

        # #longarray = np.reshape(y,(1,nelements))
        # longarray = y.ravel()
        # self.y_array.append(longarray)
        # y_array = np.asarray(self.y_array)
            newimg = []

            # Filter requirements.
            order = 2
            fs = 10       # sample rate, Hz
            cutoff = fs/10
            b, a = lowpass.butter_lowpass(cutoff, fs, order)

            if n >1:
                for i in range (0,nelements-1):
                    try:
                        x_lpf = lowpass.butter_lowpass_filter(longarray[:,i],
                                                              cutoff,
                                                              fs,
                                                              order)
                    except:
                        rospy.logwarn('Low pass filter error. Array size is %s',n)
            #     pix = x_lpf[len(x_lpf)-1]
            #     # print x_lpf.shape
            #     # print len(pix)
            #     newimg.append(pix)
            #     #x_lpf = []
            # newimg.append(0)
            # newimg = np.asarray(newimg)
            # newimg = np.reshape(newimg,(h,w,1))
            # newimg = np.float32(newimg)
            # u = np.float32(u)
            # v = np.float32(v)
            # merged = cv2.merge((newimg,u,v))
            # merged = np.float32(merged)
            # color = cv2.cvtColor(merged,cv2.COLOR_YUV2BGR)
            #
            # self.lowpass_queue.put(color)



    def yuv_equalization(self,img):
        '''
        adaptive histogram equalization over Y channel
        El modelo Y'UV consiste en una componente de luma y dos componentes
        de crominancia (UV)
        '''
        img  = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)

        y = img[:, :, 0]
        u = img[:, :, 1]
        v = img[:, :, 2]

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl1 = clahe.apply(y)

        mergedclahe = cv2.merge((cl1,u,v))

        colorclahe = cv2.cvtColor(mergedclahe,cv2.COLOR_YUV2BGR)

        self.equalization_queue.put(colorclahe)


    def homomorphic_filter(self, img):
        '''
        Code adapted from here :
        https://sites.google.com/site/bazeilst/tutorials#TUTO9
        As adapted from here :
        https://dsp.stackexchange.com/questions/42476/homomorphic-filter-python-overflow
        '''
        rospy.logdebug('Entering homomorphic filter')
        start = time.time()
        sstart = time.time()
        rows,cols = img.shape

        rh, rl, cutoff = 2.5,0.5,32

        y  = img

        y_log = np.log(y+0.01)
        tconvert = time.time() - start
        rospy.logwarn("time to compute first log %s", tconvert )


        start = time.time()
        y_fft = np.fft.fft2(y_log)
        tconvert = time.time() - start
        rospy.logwarn("time to compute first fft %s", tconvert )

        start = time.time()
        y_fft_shift = np.fft.fftshift(y_fft)
        tconvert = time.time() - start
        rospy.logwarn("time to compute second fft %s", tconvert )

        start = time.time()
        DX = cols/cutoff
        G = np.ones((rows,cols))
        for i in range(rows):
            for j in range(cols):
                G[i][j]=((rh-rl)*(1-np.exp(-((i-rows/2)**2+(j-cols/2)**2)/(2*DX**2))))+rl

        tconvert = time.time() - start
        rospy.logwarn("time to compute for loop %s", tconvert )

        start = time.time()
        result_filter = G * y_fft_shift
        tconvert = time.time() - start
        rospy.logwarn("time to compute result filter %s", tconvert )

        start = time.time()
        result_interm = np.real(np.fft.ifft2(np.fft.ifftshift(result_filter)))
        tconvert = time.time() - start
        rospy.logwarn("time to compute third fft %s", tconvert )

        start = time.time()
        result = np.exp(result_interm)
        y = np.float32(result)
        # r = np.float32(cr)
        # b = np.float32(cb)


        tconvert = time.time() - start
        rospy.logwarn("time to convert image back %s", tconvert )

        tconvert = time.time() - sstart
        rospy.logwarn("time to DO ALL %s", tconvert )
        self.homomorphic_queue.put(y)





    def crop(self, img):
        mask = np.ones(img.shape[:2], np.uint8)*255
        h, w = img.shape[:2]

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
