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
        linear_noise = 1.0 #add some noise
        #  if doesn't exist, use odom
        if xy_theta == None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)

        # make a new particle cloud and create a bunch of particles
        self.particle_cloud = []
        for x in range(self.num_particles):
            #x = random.randrange(0, width)
            #y = random.randrange(0, height)
            x = xy_theta[0]+(random_sample()*linear_noise-(linear_noise/2.0))
            y = xy_theta[1]+(random_sample()*linear_noise-(linear_noise/2.0))
            theta = math.radians(random.randrange(0, 360))
        self.particle_cloud.append(Particle(x, y, theta))

    def particle_resampler(self):
        '''Resample the particles according to the new particle weights.'''
        # make sure the distribution is normalized
        if len(self.particle_cloud):
            '''Make sure the particle weights sum to 1'''
            weights_sum = [particle.weight for particle in self.particle_cloud]

            return list(np.random.choice(self.particle_cloud, size=len(self.particle_cloud), replace=True, p=weights_sum))
        else:
            print("nothing here yet")
            return None

    def particle_normalizer(self):
        '''Make sure the particle weights sum to 1'''
        weights_sum = sum(particle.weight for particle in self.particle_cloud)
        for particle in self.particle_cloud:
            particle.weight /= weights_sum

    def particle_publisher(self, msg):
        particles = []
        for particle in self.particle_cloud:
            particles.append(particle.particle_to_pose())
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                                          frame_id="map"),
                                            poses=particles))


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
