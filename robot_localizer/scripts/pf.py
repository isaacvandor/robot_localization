#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped
from std_msgs.msg import Header, String
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import numpy as np
from numpy.random import random_sample
from particle import Particle
import random
import math


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """

    def __init__(self):
        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)


        # create instances of two helper objects that are provided to you
        # as part of the project
        self.num_particles = 500          # # of particles to use
        self.odom_pose = PoseStamped()
        self.transform_helper = TFHelper()
        self.particle_cloud = []

    def particle_cloud_init(self, xy_theta=None):
        '''
        Initialize the particle cloud
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                particle cloud around. If None, odometry will be used instead
        '''
        print("Im here now bitch")
        linear_noise = 1.0 #add some noise
        #  if doesn't exist, use odom
        if xy_theta == None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)

        for x in range(self.num_particles):
            x = xy_theta[0]+(random_sample()*linear_noise-(linear_noise/2.0))
            y = xy_theta[1]+(random_sample()*linear_noise-(linear_noise/2.0))
            theta = math.radians(random.randrange(0, 360))
            particles = Particle(x,y,theta)
        self.particle_cloud.append(particles)

    def particle_normalizer(self):
        '''Make sure the particle weights sum to 1'''
        weights_sum = sum(particle.w for particle in self.particle_cloud)
        for particle in self.particle_cloud:
            particle.w /= weights_sum

    def particle_publisher(self, msg):
        particles = []
        for p in self.particle_cloud:
            particles.append(p.particle_to_pose())
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id="odom"),
                                            poses=particles))
