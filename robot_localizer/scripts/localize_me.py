#!/usr/bin/env python

''' Gretchen and Isaac's Robot Localizer '''

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud
from helper_functions import TFHelper, convert_rotation_tr
from occupancy_field import OccupancyField
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from copy import deepcopy
import tf.transformations as t
from tf import TransformListener
from tf import TransformBroadcaster
import numpy as np
from sklearn.neighbors import NearestNeighbors
import math

class particle(object):
    ''' Represents a particle consisting of x,y coordinates, theta, and weight'''
    def __init__(self, x=0.0, y=0.0, theta=0.0,w=1.0):
        '''
        x: x-coord of the particle relative to map
        y: y-coord of the particle relative to map
        theta: angle of the particle relative to map
        w: weight of particle
        '''
        self.initialized = False # dont init anything until im ready
        rospy.init_node('localizer')
        self.base_frame = "base_link"
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.scan_topic = "stable_scan"
        self.particles = 500 #based on talking to Paul
        self.linear_thresh = 0.25 #linear movement before updating
        self.angular_thresh = math.pi/10 #angular movement before updating
        self.particle_cloud = []
        self.current_odom_xy_theta = []


        # From Paul's occupancy_field.py code
        rospy.wait_for_service('static_map')
        map_server = rospy.ServiceProxy('static_map', GetMap)
        map = map_server().map

        # initialize occupancy field
        self.occupancy_field = OccupancyField(map)
        print "Oh yeah we're occuped"
        self.initialized = True
        self.x = x
        self.y = y 
        self.theta = theta
        self.w = w 

    def odom_callback(self, msg):


    def particle_to_pose(self):
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))


    def particle_updater(self, old_pose, new_pose):
    	#create new particle cloud based on movement of robo from one position to the next
    	#update the x and y based on the new robot position
    	#update the new angle based on angle robot traveled

    	x_diff = new_pose.x - old_pose.x
    	self.x = self.x + diff

    	y_diff = new_pose.y - old_pose.y
    	self.y = self.y + diff

    	theta_diff = new_pose.theta - old_pose.theta
    	self.theta = self.theta + diff

        self.normalize_particles()

    def normalize_particles(self):
    	#do some stuff

class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
    
    def laserCallback(self, msg):
        self.stable_scan = msg
    
    def resample_particles(self):
        ''' resamples particles according to new weights which are updated
            based on laser scan messages
        '''

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def filter(self):
		#How are we filtering?
		#Look at weights and get rid of those with weight below certain range?

		#start off randomly, build from there


    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


class RunRobot(object):
    ''' Represents all of the sensor and robot model related operations'''
    def __init__(self):
    	self.transform_helper = TFHelper()
    	self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

    
    def laserCallback(self, msg):
        ''' Represents all of the logic for handling laser messages'''
        # Set ranges for front, back, left, and right
        front_left = msg.ranges[0:90]
        back_left  = msg.ranges[90:180]
        back_right   = msg.ranges[180:270]
        front_right  = msg.ranges[270:360]

        laser_diff = 0 #set difference between quadrants = 0

        # Calculate difference between left front and back to determine where wall is on left side
        for front_left, back_left in zip(front_left, reversed(back_left)):
            if front_left == 0.0 or back_left == 0.0:
                continue
            laser_diff += front_left - back_left

        # Calculate diff between right front and back to determine where wall is on right side
        for front_right, back_right in zip(front_right, reversed(back_right)):
            if front_right == 0.0 or back_right == 0.0:
                continue
            laser_diff += back_right - front_right   

	def robot_position(self):
		'''Represents the position of the robot as a geopose'''

		#read from odom
		#convert self.transform_helper.convert_translation_rotation_to_pose(self, translation, rotation)

		#odom to geopose to xyz

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
