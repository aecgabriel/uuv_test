#!/usr/bin/env python

import cv2
import numpy
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Launch():


    def __init__(self):

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image", Image, self.camera_callback)
        self.close = False
        self.ready = False
        self.height, self.width = 492, 768

    def camera_callback(self, data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            self.ready = True

        except CvBridgeError as e:

            self.close = True
            rospy.logfatal(e)

    def get(self):
        if (self.ready):

            return self.cv_image

        else:

            empty_img = numpy.zeros((self.height, self.width, 3), numpy.uint8)

            return empty_img

    def is_ready(self):
        return self.ready


def main():

    rospy.init_node("camera_node", anonymous=True)

    while(not rospy.is_shutdown()):
        rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()