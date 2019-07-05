#!/usr/bin/env python

import cv2
import numpy
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
            self.img_L = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.img_L = cv2.cvtColor(self.img_L, cv2.COLOR_BGR2GRAY)
            self.L_ready = True

        except CvBridgeError as e:
            rospy.logfatal(e)
    
    def imgR_callback(self, data):
        try:
            self.img_R = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.img_R = cv2.cvtColor(self.img_R, cv2.COLOR_BGR2GRAY)
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

        imgL = self.getL()
        cv2.imshow("Left img", imgL)
        cv2.waitKey(3)
    
    def stereo(self):

        imgL = self.getL()
        imgR = self.getR()

        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=35)
        disparity = stereo.compute(imgL,imgR)
        plt.imshow(disparity, "gray")
        plt.draw()
        plt.pause(0.01)
        #plt.imsave("/home/gabriel/Desktop/teste.png", disparity, cmap="gray")



def main():
    run = Stereo()

    while(not rospy.is_shutdown()):
        run.show_img()
        run.stereo()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
