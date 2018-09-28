#!/usr/bin/env python

''' Gretchen and Isaac's Robot Localizer '''

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, PointCloud
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from nav_msgs.srv import GetMap
from copy import deepcopy

class particle(object):
    ''' Represents a particle consisting of x,y coordinates, theta, and weight'''
    def __init__(self, x=0.0, y=0.0, theta=0.0,w=1.0)

class ParticleFilter(object):
    ''' Represents all of the particle filtering operations'''
    def __init__(self):

class RunRobot(object):
    ''' Represents all of the sensor and robot model related operations'''
    def __init__(self):

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
