#!/usr/bin/env python

#! /usr/bin/env python3
# Debe seguir el lado derecho
# Distancia entre el robot y muro: 30 cm ---> 0.3m

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# import numpy

class Publicador():

    def __init__(self):
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd = Twist()
        self.ctrl_c = False
        # self.front = 0
        self.forward_dist = 0
        self.right_dist = 0
        self.left_dist = 0
        self.rate = rospy.Rate(1) # 10hz
        # self.cmd.linear.x = 0.05
        self.state = 0
        self.dist = 0.4
        rospy.on_shutdown(self.shutdownhook)   
        # angular_z_+ = turn_left
        # angular_z_- = turn_right

    def move_robot(self):
        self.state = 0
        while not self.ctrl_c:
            rospy.sleep(0.5)
            if self.state == 0:
                self.cmd.angular.z = 0.0
                if self.forward_dist > 0.3:
                    self.cmd.linear.x = 0.1
                    
                else:
                    self.cmd.linear.x = 0.0
                    self.state = 1

            if self.state == 1:
                while self.right_dist > 0.3:
                    self.cmd.angular.z = 0.2
                    self.vel_publisher.publish(self.cmd)
                self.cmd.angular.z = 0
                self.vel_publisher.publish(self.cmd)
                self.state = 2 

            self.vel_publisher.publish(self.cmd)
            self.rate.sleep() #

    def scan_callback(self,data):
        # self.front = data.ranges[0]
        self.forward_dist=data.ranges[360]
        self.backward_dist=data.ranges[0] # Backward_dist
        self.left_dist=data.ranges[540]
        self.right_dist=data.ranges[180]
        print("State: ", self.state)
        print("Front: " + str(self.forward_dist) + " Back: " + str(self.backward_dist))
        print("Left: " + str(self.left_dist) + " Right: " + str(self.right_dist))
        print("Size: ", len(data.ranges))


    def shutdownhook(self):
        # trabaja mucho mejor que el rospy.is_shutdown()
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('publish_test_1', anonymous=True)
    turtlebot_object = Publicador()
    try:
        turtlebot_object.move_robot()
    except rospy.ROSInterruptException:
        pass