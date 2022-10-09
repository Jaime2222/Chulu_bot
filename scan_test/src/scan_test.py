#! /usr/bin/env python3
# Debe seguir el lado derecho
# Distancia entre el robot y muro: 30 cm ---> 0.3m
 
import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    print(len(msg.ranges))
rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()