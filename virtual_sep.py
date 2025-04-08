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

    def preprocess_lidar(self):
    #we want an array that contains all the scan distances in a "pizza slide" in front of the car, with all the distances that are too small set to zero.      
    	fov = 70 #in degrees, this is how wide we are looking on either side of the base_link x
	start_index = 2*(180 - fov)
	end_index = start_index + 2*(2*fov) #actually I guess I'm not using this one
	for i in range(4*fov): #will do this loop once for each index within the "pizza slice" wedge of space in front of car
		pizza_slice[i] = data.ranges[start_index + i]
		if pizza_slice[i] <= self.d_stop: #all distances that are too small will be set to 0
			pizza_slice[i] = 0
		else:
			continue
    return pizza_slice
             


    # Optional function to find the the maximum gap in fron the vehicle 
    def find_max_gap(self, proc_ranges):
    	max_subset = []
    	max_subset_indices = []
    	current_max_subset = []
    	current_max_subset_indices = []
	for i in len(pizza_slice): #going through the pizza slice, which is a new array, with indices starting from 0
        	if pizza_slice[i] > 0:
            		current_max_subset.append(pizza_slice[i]) #the measurement gets added to the current max array
            		current_max_subset.append(i) #the index of the measurement in the pizza slice 
        	else:
            		if len(current_max_subset) > len(max_subset): #the subset you just counted is actually the longest
                		max_subset = current_max_subset
                		max_subset_indices = current_max_subset_indices
    #now we have the array max_subset which has the distances, and max_subset_indices which has the indices of it starting counting from the lidar scan represented by start_index. 
        			max_gap_start_index = start_index + max_subset_indices[0]
        			max_gap_end_index = start_index + max_subset_indices[-1]
        			gap_indices = [max_gap_start_index, max_gap_end_index]
        return gap_indices
        

    #Optional function to find the best direction of travel
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i):
        biggest_distance = data.ranges[start_i] #ok not actually using this but oh well will remove it later
        direction_index = start_i
        for i in len(end_i - start_i):
            if data.ranges[start_i + i] > biggest:
                biggest_distance = data.ranges[start_i + i]
                direction_index = start_i + i      
        return direction_index    
            

 
    # Optional function to set up and solve the optimization problem for parallel virtual barriers 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):
     
     # ...
     

    # This function is called whenever a new set of LiDAR data is received; bulk of your controller implementation should go here 
    def lidar_callback(self, data):      

    # Pre-process LiDAR data as necessary
    	pizza_slice = preprocess_lidar() #this is an array containing lidar distance values in front of car
	#now we have an array of the distances in the FOV, and want to find the longest span of nonzeros in it
    #find_max_gap() takes in the array pizza_slice of distances, outputs the indices in the pizza slice of the max gap that then need to be added to start_index.
    	gap_indices_in_pizza = find_max_gap(pizza_slice)
    #now we want the actual lidar index corresponding to the indices in the pizza slice
    	gap_start = start_index + gap_indices_in_pizza[0]
    	gap_end = start_index + gap_indices_in_pizza[1]
    #then just need to find largest distance within that gap
    	best_direction_index = find_best_point(gap_start, gap_end)
    #now we have the index of the lidar scan in which direction we want the car to go (and its angle is half the scan index.)


        # Set up the QP for finding the two parallel barrier lines
        #want to look for obstacles from b_r to a_r
        n_r = 2*self.a_r - 2*self.b_r
        n_l = 2*self.b_l - 2*self.a_l
        for ray in range(n_r):
            right_obstacle_angle = np.radians(self.b_r + (ray/2))
            right_obstacle_dist[ray] = data.ranges[2*self.b_r + ray]
            p_x_r[ray] = right_obstacle_dist[ray]*np.cos(right_obstacle_angle)
            p_y_r[ray] = right_obstacle_angle[ray]*np.sin(right_obstacle_angle)
        for ray in range(n_l):
            left_obstacle_angle = np.radians(self.a_l + ray/2))
            left_obstacle_dist[ray] = data.ranges[2*self.a_l + ray]
            p_x_l[ray] = left_obstacle_dist[ray]*np.cos(left_obstacle_angle)
            p_y_l[ray] = left_obstacle_dist[ray]*np.sin(left_obstacle_angle)

            #creating the C array:
            bottom_ones = np.ones_like(p_y_r)
            bottom_neg_ones = -1*np.ones_like(p_y_l)
            left_part = np.vstack(p_x_r, p_y_r, bottom_ones)
            middle_part = np.vstack(-1*p_x_l, -1*p_y_l, bottom_neg_ones)
            right_part = np.array([0, 0], [0, 0], [1, -1])
            C = np.hstack(left_part, middle_part, right_part)
            #The other arrays are given
            G = np.array([1, 0, 0],[0, 1, 0], [0, 0, 0.0001])
            a = np.array([0], [0], [0])
            b = np.ones((n_r _ n_l + 2, 1))
            b[-1] = -0.99
            b[-2] = -0.99

            # Solve the QP problem to find the barrier lines parameters w,b
            minimized = solve_qp(G, a, C, b, meq)[0]        

            # Compute the values of the variables needed for the implementation of feedback linearizing+PD controller
            w = minimized[0:2]
            b = minimized[2]
            w_r = w/(b - 1) #should work if w is a numpy array rather than a list, but not sure if it is
            w_l = w/(b + 1)
            d_l = 1/np.sqrt(np.transpose(w_l)*w_l)
            d_r = 1/np.sqrt(np.transpose(w_r)*w_r)
            w_l_hat = d_l*w_l
            w_r_hat = d_r*w_r
            d_l_dot = np.array([self.vel, 0])*w_l_hat
            d_r_dot = np.array([self.vel, 0])*w_r_hat
            cos_al = np.array([0, -1])*w_l_hat
            cos_ar = np.array([0, 1])*w_r_hat
            dlr = d_l - d_r
            dlr_dot = d_l_dot - d_r_dot

            
                # Compute the steering angle command
            if self.vel == 0: #need this to avoid division by 0 in the next line, but not sure about it
		        delta = 0
		        steering_angle = 0
	        else:
		        delta = np.arctan((-1*self.l*(-self.k_p*dlr - self.k_d*dlr_dot))/((self.vel**2)*(np.cos(alpha_r) + np.cos(alpha_l))))
                	if delta > self.delta_max:
                    		steering_angle = self.delta_max
                	elif -self.delta_max <= delta and delta <= self.delta_max:
                    		steering_angle = delta
                	elif delta < -self.delta_max:
                    		steering_angle = -self.delta_max
                	else:
			        steering_angle = 0
                    		print("Error: something wrong with steering angle")
                
            # Find the closest obstacle point in a narrow field of view in fronnt of the vehicle and compute the velocity command accordingly    
            starting_index = int(2*(180 - self.delta_theta/2))
            ending_index = int(starting_index + 2*self.delta_theta)
            pizza_slice_2 = data.ranges[starting_index:ending_index:1]
            d_ob = np.min(pizza_slice_2)
            speed = self.vs_d*(1 - np.exp(-np.max(d_ob - self.d_stop, 0)/self.d_tau))
                
            # Publish the steering and speed commands to the drive topic
            self.drive_topic = AckermannDriveStamped()
            self.drive_topic.drive.steering_angle = steering_angle
            self.drive_topic.drive.speed = speed
            self.drive_topic.header.stamp = rospy.Time.now()
            self.drive_pub.publish(self.drive_topic)


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
