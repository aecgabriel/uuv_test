#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt


class Stereo():

    def __init__(self):

        rospy.init_node("Stereo_node", anonymous=True)
        self.imgL_sub = rospy.Subscriber("/rexrov/rexrov/cameraleft/camera_image", Image, self.imgL_callback)
        self.imgR_sub = rospy.Subscriber("/rexrov/rexrov/cameraright/camera_image", Image, self.imgR_callback)
        self.bridge = CvBridge()
        self.R_ready = False
        self.L_ready = False
        self.height, self.width = 492, 768

    def imgL_callback(self, data):
        try:
            input_L = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.img_L = cv2.cvtColor(input_L, cv2.COLOR_BGR2GRAY)
            self.L_ready = True

        except CvBridgeError as e:
            rospy.logfatal(e)
    
    def imgR_callback(self, data):
        try:
            input_R = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.img_R = cv2.cvtColor(input_R, cv2.COLOR_BGR2GRAY)
            self.R_ready = True

        except CvBridgeError as e:
            rospy.logfatal(e)
    
    def getL(self):
        if (self.L_ready):
            return self.img_L
        else:
            empty_img = cv2.imread("/home/gabriel/Documents/Repositorios/opencv/samples/data/left01.jpg", 0)
            return empty_img
    
    def getR(self):
        if (self.R_ready):
            return self.img_R
        else:
            empty_img = cv2.imread("/home/gabriel/Documents/Repositorios/opencv/samples/data/right01.jpg", 0)
            return empty_img

    def show_img(self):

        cv2.imshow('Disparity Map', self.filteredImg)
        cv2.imshow("Left img", self.getL())
        cv2.waitKey(3)
    
    def stereo(self):
        
        imgL = self.getL()
        imgR = self.getR()

        #stereo = cv2.StereoBM_create(numDisparities=16, blockSize=35)
        #disparity = stereo.compute(imgL,imgR)
        #plt.imshow(disparity, "gray")
        #plt.draw()
        #plt.pause(0.01)

        # SGBM Parameters -----------------
        window_size = 3                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

        left_matcher = cv2.StereoSGBM_create(
            minDisparity=-64,
            numDisparities=192,             # max_disp has to be dividable by 16 f. E. HH 192, 256
            blockSize=5,
            P1=600,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
            P2=2400,
            disp12MaxDiff=10,
            uniquenessRatio=1,
            speckleWindowSize=150,
            speckleRange=2,
            preFilterCap=4,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

        # FILTER Parameters
        lmbda = 80000
        sigma = 1.2

        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)

        print('computing disparity...')
        displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
        dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
        displ = np.int16(displ)
        dispr = np.int16(dispr)
        self.filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
        self.filteredImg = cv2.normalize(src=self.filteredImg, dst=self.filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        self.filteredImg = np.uint8(self.filteredImg)
        


def main():
    run = Stereo()

    while(not rospy.is_shutdown()):
        run.stereo()
        run.show_img()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
