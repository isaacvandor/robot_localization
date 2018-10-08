#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from std_msgs.msg import Header, String
from helper_functions import TFHelper
from occupancy_field import OccupancyField
import numpy as np
from particle import Particle
import random
import math


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """

    def __init__(self):
#        rospy.init_node('pf')

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
        self.num_particles = 500          # # of particles to use
        self.transform_helper = TFHelper()
        self.particle_cloud = []

    def particle_cloud_init(self, w, h, xy_theta=None):
        '''
        Initialize the particle cloud
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                particle cloud around. If None, odometry will be used instead
        '''

        # make a new particle cloud and create a bunch of particles
        self.particle_cloud = []
        width = w
        height = h
        for x in range(self.num_particles):
            x = random.randrange(0, width)
            y = random.randrange(0, height)
            theta = math.radians(random.randrange(0, 360))
        self.particle_cloud.append(Particle(x, y, theta))

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

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()

    def particle_resampler(self):
        '''Resample the particles according to the new particle weights.'''
        # make sure the distribution is normalized
        if len(self.particle_cloud):
            '''Make sure the particle weights sum to 1'''
            weights_sum = sum(
                particle.weight for particle in self.particle_cloud)
            for particle in self.particle_cloud:
                particle.weight /= weights_sum

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
