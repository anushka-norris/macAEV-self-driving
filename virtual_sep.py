#!/usr/bin/env python
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read the algorithm parameter paramters form params.yaml
	lidarscan_topic =rospy.get_param('~scan_topic')
        odom_topic= rospy.get_param('~odom_topic')
	drive_topic = rospy.get_param('~nav_drive_topic')
	self.scan_beams = rospy.get_param('~scan_beams')
        self.b_l = rospy.get_param('~b_l')
        self.b_r = rospy.get_param('~b_r')
        self.a_l = rospy.get_param('~a_l')
        self.a_r = rospy.get_param('~a_r')
        self.k_p = rospy.get_param('~k_p')
        self.k_d = rospy.get_param('~k_d')
        self.vs_d = rospy.get_param('~vs_d')
        self.delta_theta = rospy.get_param('~delta_theta')
        self.d_stop = rospy.get_param('~d_stop')
        self.d_tau = rospy.get_param('~d_tau')
        self.delta_max = rospy.get_param('~delta_max')
        self.l = rospy.get_param('~wheelbase')        

 

        # Add your subscribers for LiDAR scan and Odomotery here
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1) 

        # Add your publisher for Drive topic here
	self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        # Initialize varables as needed 
        #...


    # Optional function to pre-process LiDAR by considering only those LiDAR returns within a FOV in front of the vehicle;    

    def preprocess_lidar(self, ranges):
        
    # ...            
             


    # Optional function to find the the maximum gap in fron the vehicle 
    def find_max_gap(self, proc_ranges):
        
    #Optional function to find the best direction of travel
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i, proc_ranges):
     



    # Optional function to set up and solve the optimization problem for parallel virtual barriers 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):
     
     # ...
     

    # This function is called whenever a new set of LiDAR data is received; bulk of your controller implementation should go here 
    def lidar_callback(self, data):      

    # Pre-process LiDAR data as necessary
    # ...
    
    	# Find the widest gape in front of vehicle
	fov = 70 #in degrees, this is how mide we are looking on either side of the base_link x
	start_index = 2*(180 - fov)
	end_index = start_index + 2*(2*fov)
	for i in range(4*fov): #will do this loop once for each index within the pizza slice
		distance[i] = data.ranges[start_index + i]
		if distance <= self.d_stop: #all distances that are too small will be set to 0
			distance = 0
		else
			continue
	#now we have an array of the distances in the FOV, and want to find the longest span of nonzeros in it
	
		

    
    # Find the Best Direction of Travel
    # ...

    # Set up the QP for finding the two parallel barrier lines
    # ...

    # Solve the QP problem to find the barrier lines parameters w,b

    # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
    # ...
    
    # Compute the steering angle command
        
    # Find the closest obstacle point in a narrow field of view in fronnt of the vehicle and compute the velocity command accordingly    
    # ...
        
    # Publish the steering and speed commands to the drive topic
    # ...


    # Odometry callback 
    def odom_callback(self, odom_msg):
        # update current speed
         self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
