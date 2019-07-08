#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pow,sqrt,pi

class GoTo:

    def __init__(self):
        
        #Subscribe to the Point node to receive the target
        self.target_sub = rospy.Subscriber('Point', Point, self.target)
        #Subscribe to the pose_gt node to receive the robot's base positon
        self.real_pose = rospy.Subscriber('pose_gt', Odometry, self.update_position)
        #Publish to the cmd_vel to send the linear and angular robot's speed
        self.speed_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.goal = [0, 0, -.5]
        self.pose = [0, 0, 0]
        (self.roll, self.pitch, self.theta) = (0, 0, 0)
        self.rotation = 0
        self.distance_tolerance = 0.05
        self.angle_tolerance = 0.005


    def update_position(self,msg):
        #Receives the data from the pose_gt topic
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        self.pose[2] = msg.pose.pose.position.z
        self.rotation = msg.pose.pose.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion([self.rotation.x,
         self.rotation.y, self.rotation.z, self.rotation.w])

    def target(self,msg):
        #Receives de data from the Point topic
        self.goal[0] = msg.x
        self.goal[1] = msg.y
        self.goal[2] = msg.z     
    

    def run(self):
        #Creates the speed variable with the Twist type to publish on cmd_vel topic
        speed = Twist()
        #Calculates the angle with the arc tangent of X-Y considering the signals
        angle =  atan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0])
        #Calculates the difference between the robot's angle and the goal angle
        error_angle = angle - self.theta
        #Calculates the difference between the robot's position and the goal
        error_x = self.goal[0] - self.pose[0]
        error_y = self.goal[1] - self.pose[1]
        error_z = self.goal[2] - self.pose[2] 

        if(error_angle > pi):
            error_angle -= 2*(pi)
        elif error_angle < -pi:
            error_angle += 2*(pi)

        speed.angular.z = error_angle*.5
        if speed.angular.z > .5:
            speed.angular.z = .5
        elif speed.angular.z < -.5:
            speed.angular.z = -.5


        if abs(error_x or error_y) > self.distance_tolerance:
            speed.linear.x = abs(error_x)*.5
            if speed.linear.x > 1:
                speed.linear.x = 1
        else:
            speed.linear.x = 0
            speed.angular.z = 0
        
        if error_z > self.distance_tolerance or error_z < -self.distance_tolerance:
            speed.linear.z = error_z
            if error_z > 1:
                speed.linear.z = 1
            elif error_z < -1:
                speed.linear.z = -1
        else:
            speed.linear.z = 0

        self.speed_pub.publish(speed)

if __name__ == "__main__":
    #Iniciate the node and ensure that's unique with the anonymous
    rospy.init_node("speed_controller", anonymous=True)
    #The variable goTo receives the GoTo class
    goTo = GoTo()
    #The rate variable receives a frequency in Hz 
    rate = rospy.Rate(10)
    
    while(not rospy.is_shutdown()):
        #Run the program
        goTo.run()
        #To avoid overflow during the while the sleep is used
        rate.sleep()