#!/usr/bin/env python3
import rospy
#import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int64,Int32
from nav_msgs.msg import Odometry
import sys
import tf

class line_follower:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.vel = Twist()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.callback_odom)
        self.direc_sub1 = rospy.Subscriber("direction", Int32, self.mask_callback1)
        # self.direc_sub2 = rospy.Subscriber("mask2", Int32, self.mask_callback2)
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.callback_laser)
        # Some variables
        self.flag_hy = False
        self.prev_flag_hy = False
        self.counter = 0
        self.state = 0

    def callback_odom(self,data):
        self.x=data.pose.pose.position.x
        self.y=data.pose.pose.position.y
        self.abs = (self.x**2+self.y**2)**0.5
        self.z=data.pose.pose.position.z
        self.euler=tf.transformations.euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
        self.yaw = self.euler[2]

    def callback_laser(self, data):
        self.forward_dist=data.ranges[360]
        self.needed=data.ranges[430]
        self.left_dist=data.ranges[719]
        self.right_dist=data.ranges[0]
        print("dist: ",self.forward_dist)
        # print("Needed: ",self.needed)
        print("state", self.state)

    def mask_callback1(self,data):
        self.error = data.data
        if self.state == 0:
            if self.forward_dist<1:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                self.state = 1
            else:
                self.vel.linear.x = 0.5
                self.pub.publish(self.vel) 
                if self.error == -100000:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0.3
                    self.pub.publish(self.vel) 
                else:
                    self.vel.angular.z = 0.01*self.error
                    self.pub.publish(self.vel) 

        if self.state == 1:
            if self.abs<6.5:
                if self.needed < 4:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0.5 # turn right
                    self.pub.publish(self.vel) 
                else:
                    self.vel.linear.x = 0.5
                    self.vel.angular.z = 0 # turn right
                    self.pub.publish(self.vel) 
            else:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
                self.pub.publish(self.vel)
                self.state = 2

        if self.state == 2:
            self.vel.linear.x = 0.5
            self.pub.publish(self.vel) 
            if self.error == -100000:
                self.vel.linear.x = 0
                self.vel.angular.z = 0.3
                self.pub.publish(self.vel) 
            else:
                self.vel.angular.z = 0.01*self.error
                self.pub.publish(self.vel) 


def main(args):
    rospy.init_node('line_follower', anonymous=True)
    ms = line_follower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)