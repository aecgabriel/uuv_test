#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

class Joystick_control():

    def __init__(self):
        
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.linear = Vector3(0,0,0)
        self.angular = Vector3(0,0,0)
        self.deadman = 0
    
    def joy_callback(self,data):

        self.angular.z = data.axes[0]*.5
        self.linear.x = data.axes[1]*.5
        self.linear.y = data.axes[2]*.5
        self.linear.z = data.axes[3]*.5
        self.deadman = data.buttons[5]

    def control(self):
        
        ctrl = Twist()

        if self.deadman != 1:
            ctrl.linear = self.linear
            ctrl.angular = self.angular

        
        self.cmd_pub.publish(ctrl)
    
def start():
    run= Joystick_control()

    rospy.init_node('Joy_controller', anonymous=True)
    
    while(not rospy.is_shutdown()):
        run.control()


        

if __name__ == "__main__":
    start()    