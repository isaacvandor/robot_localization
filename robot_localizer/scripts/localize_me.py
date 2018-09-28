#!/usr/bin/env python

''' Gretchen and Isaac's Robot Localizer '''

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud
from helper_functions import TFHelper, convert_rotation_tr
from occupancy_field import OccupancyField
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
        self.x = x
        self.y = y 
        self.theta = theta
        self.w = w 

    def particle_to_pose(self):
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))
class ParticleFilter(object):
    ''' Represents all of the particle filtering operations'''
    def __init__(self):


class RunRobot(object):
    ''' Represents all of the sensor and robot model related operations'''
    def __init__(self):
    
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

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
