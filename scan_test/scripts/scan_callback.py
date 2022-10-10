#!/usr/bin/env python

#! /usr/bin/env python3
# Debe seguir el lado derecho
# Distancia entre el robot y muro: 30 cm ---> 0.3m

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Publicador():

    def __init__(self):
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd = Twist()
        self.ctrl_c = False
        # self.front = 0
        self.target = 0.4
        self.kp = 1.5
        self.error = 0
        self.forward_dist = 0
        self.right_dist = 0
        self.needed_left = 0
        self.needed_right = 0
        self.left_dist = 0
        self.rate = rospy.Rate(1) # 1hz
        # self.state = ""
        self.state = "follow_wall"
        self.dist = 0.4
        rospy.on_shutdown(self.shutdownhook)   
        # angular_z_+ = turn_left
        # angular_z_- = turn_right

    def move_robot(self):
        while not self.ctrl_c:
            # self.state = "go_to_wall"
            rospy.sleep(1)
            if self.state == "go_to_wall":
                self.cmd.angular.z = 0.0
                if self.forward_dist > 0.3:
                    self.cmd.linear.x = 0.1
                else:
                    self.cmd.linear.x = 0.0
                    self.state = "wall_right_side"

            if self.state == "wall_right_side":
                if self.right_dist > 0.3 and self.needed_left > 0.1:
                    self.cmd.angular.z = 0.2
                    self.vel_publisher.publish(self.cmd)
                else:
                    self.cmd.angular.z = 0
                    self.vel_publisher.publish(self.cmd)
                    self.state = "follow_wall"       
            
            if self.state == "follow_wall":
                 
                if self.forward_dist > 0.3:
                    self.cmd.linear.x = 0.08
                    self.cmd.angular.z = self.kp*self.error
                    self.vel_publisher.publish(self.cmd)
                    print("follow_wall")
                else:
                    self.cmd.linear.x = 0
                    self.cmd.angular.z = 0
                    self.vel_publisher.publish(self.cmd)
                    self.state = "wall_right_side"


                #if self.right_dist > 0.3:
                #    self.cmd.linear.x = 0
                #    self.cmd.angular.z = -0.3
                #    self.vel_publisher.publish(self.cmd)
#
                #elif self.right_dist < 0.2:
                #    self.cmd.linear.x = 0
                #    self.cmd.angular.z = 0.3
                #    self.vel_publisher.publish(self.cmd)
#
                #if self.right_dist == 0.2 and self.right_dist == 0.3:
                #    self.cmd.angular.z = 0    
                #    self.cmd.linear.x = 0.5
                #    self.vel_publisher.publish(self.cmd) 
#
                #if self.forward_dist < 0.5:
                #    self.cmd.angular.z = 0.5    
                #    self.cmd.linear.x = 0.3
                #    self.vel_publisher.publish(self.cmd)     


            #if self.state == 0:
            #    self.cmd.angular.z = 0.0
            #    if self.forward_dist > 0.5:
            #        self.cmd.linear.x = 0.1
            #    else:
            #        self.cmd.linear.x = 0.0


            self.vel_publisher.publish(self.cmd)
            self.rate.sleep() # 


    def scan_callback(self,data):
        self.forward_dist = data.ranges[360]
        self.backward_dist = data.ranges[0] # Backward_dist
        self.left_dist = data.ranges[540]
        self.right_dist = data.ranges[180]
        self.needed_left = data.ranges[450]
        self.needed_right = data.ranges[260]
        self.error = self.target - self.needed_right
        
        print("State: ", self.state)
        print("Front: " + str(self.forward_dist) + " Back: " + str(self.backward_dist))
        print("Left: " + str(self.left_dist) + " Right: " + str(self.right_dist))
        print("Needed_Left: " + str(self.needed_left) + " Needed_Right: " + str(self.needed_right))
        print("Error: ", self.error)

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
