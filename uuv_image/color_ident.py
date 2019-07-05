#!/usr/bin/env python

import cv2
import numpy
import rospy
import camera


class color_ident():

    def __init__(self):
        self.start = camera.Launch()

    def convert_to_HSV(self):

        self.hsv = cv2.cvtColor(self.start.cv_image, cv2.COLOR_BGR2HSV)
        self.lower_blue = numpy.array([110, 50, 50])
        self.upper_blue = numpy.array([130, 255, 255])

    def apply_mask(self):

        self.mask = cv2.inRange(self.hsv, self.lower_blue, self.upper_blue)
        self.res = cv2.bitwise_and(self.start.cv_image, self.start.cv_image, mask=self.mask)

    def show_img(self):
        img = self.start.get()
        rospy.loginfo_throttle(60, "Camera started")
        cv2.imshow("Rexrov camera", img)
        cv2.imshow("Res camera", self.res)

        cv2.waitKey(3)


def main():

    rospy.init_node("camera_node", anonymous=True)
    run = color_ident()

    while(not rospy.is_shutdown()):

        if(run.start.is_ready()):

            run.convert_to_HSV()
            run.apply_mask()
            run.show_img()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()