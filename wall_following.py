#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry




class WallFollow:
    def __init__(self):

        # Read the Wall-Following controller paramters form params.yaml
        lidarscan_topic =rospy.get_param('~scan_topic')
        odom_topic= rospy.get_param('~odom_topic')
	drive_topic = rospy.get_param('~nav_drive_topic')
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
        
    
        # Subscrbie to LiDAR scan Wheel Odometry topics. This is to read the LiDAR scan data and vehicle actual velocity
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Create a publisher for the Drive topic
        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        

     # The LiDAR callback function is where you read LiDAR scan data as it becomes availble and compute the vehile veloicty and steering angle commands
    
    def lidar_callback(self, data):      

      # Exttract the parameters of two walls on the left and right side of the vehicles. Referrring to Fig. 1 in the lab instructions, these are al, bl, thethal, ...   
        #get distances of a, b, etc
        a_r_index = 2*self.a_r
        b_r_index = 2*self.b_r
        a_l_index = 2*self.a_l
        b_l_index = 2*self.b_l
        a_r_distance = data.ranges[a_r_index]
        b_r_distance = data.ranges[b_r_index]
        a_l_distance = data.ranges[a_l_index]
        b_l_distance = data.ranges[b_l_index]

      # Compute the steering angle command to maintain the vehicle in the middle of left and and right walls
	a_r = np.radians(self.a_r)
	a_l = np.radians(self.a_l)
	b_r = np.radians(self.b_r)
	b_l = np.radians(self.b_l) 
	theta_r = a_r - b_r
        theta_l = b_l - a_l
        beta_r = np.arctan2((a_r_distance*np.cos(theta_r) - b_r_distance), (a_r_distance*np.sin(theta_r)))
        beta_l = np.arctan2((a_l_distance*np.cos(theta_l) - b_l_distance), (a_l_distance*np.sin(theta_l)))
        d_r = b_r_distance*np.cos(beta_r)
        d_l = b_l_distance*np.cos(beta_l)
        alpha_r = beta_r + np.pi/2 - b_r
        alpha_l = -1*beta_l + 3*np.pi/2 - b_l
        dlr = d_l - d_r
        dlr_dot = -1*self.vel*np.sin(alpha_l) - self.vel*np.sin(alpha_r)
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
            		print("Error: something wrong with steering angle")

        
      # Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
        starting_index = int(2*(180 - self.delta_theta/2))
        ending_index = int(starting_index + 2*self.delta_theta)
        pizza_slice = data.ranges[starting_index:ending_index:1]
        d_ob = np.min(pizza_slice)
        speed = self.vs_d*(1 - np.exp(-np.max(d_ob - self.d_stop, 0)/self.d_tau))

      # Publish steering angle and velocity commnads to the Drive topic
        self.drive_topic = AckermannDriveStamped()
        self.drive_topic.drive.steering_angle = steering_angle
        self.drive_topic.drive.speed = speed
        self.drive_topic.header.stamp = rospy.Time.now()
        self.drive_pub.publish(self.drive_topic)


    # The Odometry callback reads the actual vehicle velocity from VESC. 
    
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x
#	self.vel = 1


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
