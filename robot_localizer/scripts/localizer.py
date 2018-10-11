#!/usr/bin/env python

from __future__ import print_function
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Point, PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from particle import Particle
import statistics
from occupancy_field import OccupancyField
from copy import deepcopy
import random as r
import time, numpy, math, rospy
from helper_functions import TFHelper
from tf import TransformListener
from tf import TransformBroadcaster
import numpy as np
from numpy.random import random_sample
from pf import ParticleFilter


class RobotLocalizer(object):
    '''
    The class that represents a Particle Filter ROS Node
    '''

    def __init__(self):
        print('Initializing')
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('localizer')
        self.pf = ParticleFilter()

        self.base_frame = "base_link"   # Robot base frame
        self.map_frame = "map"          # Map coord frame
        self.odom_frame = "odom"        # Odom coord frame
        self.scan_topic = "scan"        # Laser scan topic

        self.num_particles = 100          # # of particles to use

        self.linear_threshold = 0.1             # the amount of linear movement before performing an update
        self.angular_threshold = math.pi/10     # the amount of angular movement before performing an update

        self.max_dist = 2.0   # maximum penalty to assess in the likelihood field model

        self.odom_pose = PoseStamped()
        self.robot_pose = Pose()

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        # init pf
        # subscribers and publisher
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_particle_updater)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.pose_updater)
        # enable listening for and broadcasting coord transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.current_odom_xy_theta = []

        print("initialization complete")
        self.initialized = True

    def robot_pose_updater(self):
        ''' Update the estimate of the robot's pose given the updated particles by computing the mean pose'''

        self.pf.particle_normalizer()

        # Calculate avg particle position based on pose
        mean_particle = Particle(0, 0, 0, 0)
        mean_particle_theta_x = 0
        mean_particle_theta_y = 0
        for particle in self.pf.particle_cloud:
            mean_particle.x += particle.x * particle.w
            mean_particle.y += particle.y * particle.w

            # Using trig to calculate angle (@Paul I hate Trigonometry!)
            distance_vector = np.sqrt(np.square(particle.y)+ np.square(particle.x))
            mean_particle_theta_x += distance_vector * np.cos(particle.theta) * particle.w
            mean_particle_theta_y += distance_vector * np.sin(particle.theta) * particle.w

        mean_particle.theta = np.arctan2(float(mean_particle_theta_y),float(mean_particle_theta_x))

        self.robot_pose = mean_particle.particle_to_pose()

    # Get the laser messages
    def laserCallback(self, msg):
        self.laserCallback = msg

    def odom_particle_updater(self, msg):
        ''' Updates particles based on new odom pose using a delta value for x,y,theta'''
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change (delta) in x,y,theta
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        odom_noise = .25 # noise level

        '''
        updates the particles based on angle1, dist, and angle2.
        angle1: Angle by which the robot rotates to face new xy position
        dist: Distance to move forward to xy position
        angle2: Angle by which the robot rotates to face final direction
        '''
        for particle in self.pf.particle_cloud:
            # calculates angle1, d, and angle2
            angle1 = np.arctan2(float(delta[1]),float(delta[0])) - old_odom_xy_theta[2]
            dist = np.sqrt(np.square(delta[0])+np.square(delta[1]))
            angle2 = delta[2] - angle1

            # updates the particles with the above variables, while also adding in some noise
            #This is the part of class where Paul moved and we all updated based on that movement
            particle.theta = particle.theta + angle1*(random_sample()*odom_noise+(1-odom_noise/2.0))
            particle.x = particle.x + dist*np.cos(particle.theta)*(random_sample()*odom_noise+(1-odom_noise/2.0))
            particle.y = particle.y + dist*np.sin(particle.theta)*(random_sample()*odom_noise+(1-odom_noise/2.0))
            particle.theta = particle.theta + angle2*(random_sample()*odom_noise+(1-odom_noise/2.0))


    def particle_resampler(self):
        '''Resample the particles according to the new particle weights.'''
        # make sure the distribution is normalized
        self.pf.particle_normalizer()

        if len(self.pf.particle_cloud):
            '''Make sure the particle weights sum to 1'''
            weights_sum = [particle.w for particle in self.pf.particle_cloud]

            return list(np.random.choice(self.pf.particle_cloud, size=len(self.pf.particle_cloud), replace=True, p=weights_sum))
        else:
            print("help i've fallen")
            return None

    def laser_particle_updater(self, msg):
        '''Updates the particle weights in response to the scan contained in the msg'''

        # Find the total error for each particle based on 36 laser measurements taken from the Neato's actual position
        for particle in self.pf.particle_cloud:
            error = []
            for theta in range(0,360,10):
                rad = np.radians(theta)
                err = self.occupancy_field.get_closest_obstacle_distance(particle.x + msg.ranges[theta] * np.cos(particle.theta + rad), particle.y + msg.ranges[theta] * np.sin(particle.theta + rad))
                if (math.isnan(err)):   # if the get_closest_obstacle_distance method finds that a point is out of bounds, then the particle can't ever be it
                    particle.w = 0
                    break
            if (sum(error) == 0):     # if the particle is basically a perfect match, then we make the particle almost always enter the next iteration through resampling
                particle.w = 1.0
            else:
                particle.w = 1.0/sum(error)   # the errors are inverted such that large errors become small and small errors become large

    def pose_updater(self, msg):
        ''' Restart particle filter based on updated pose '''
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        #self.fix_map_to_odom_transform(msg)
        self.pf.particle_cloud_init(xy_theta)
        self.fix_map_to_odom_transform(msg)
        print("particle cloud initialized")

    def process_scan(self, msg):
        '''Handling laser data to update our understanding of the robot based on laser and odometry'''
        if not(self.initialized):
            print("first if not")
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform('base_link',msg.header.frame_id,msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform('base_link','odom',msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return
        # calculate pose of laser relative ot the robot base
        pose = PoseStamped(header=Header(stamp=rospy.Time(0),
                                      frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame,pose)

        # find out where the robot thinks it is based on its odometry
        pose = PoseStamped(header=Header(stamp=msg.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, pose)
        # store the the odometry pose in a more convenient format (x,y,theta)
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        if not(self.pf.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.pf.particle_cloud_init()
            # cache the last odometry pose so we can only update our particle filter if we move more than self.linear_threshold or self.angular_threshold
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            print("Trying to initialize!")
            self.fix_map_to_odom_transform(msg)
            print("Initialized finally!")
            self.pf.particle_publisher(msg)
        else:
            # we have moved far enough to do an update!
            self.odom_particle_updater(msg)    # update based on odometry
            #print("map!")
            self.laser_particle_updater(msg)   # update based on laser scan
            self.robot_pose_updater()                # update robot's pose
            self.particle_resampler()               # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(msg)     # update map to odom transform now that we have new particles
            self.pf.particle_publisher(msg)

    def fix_map_to_odom_transform(self, msg):
            """ This method constantly updates the offset of the map and
                odometry coordinate systems based on the latest results from
                the localizer """
            (translation, rotation) = \
                self.transform_helper.convert_pose_inverse_transform(self.robot_pose)
            pose = PoseStamped(pose=self.transform_helper.convert_translation_rotation_to_pose(translation,rotation),
                            header=Header(stamp=msg.header.stamp,frame_id=self.base_frame))
            self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, msg.header.stamp, rospy.Duration(1.0))
            self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, pose)
            (self.translation, self.rotation) = self.transform_helper.convert_pose_inverse_transform(self.odom_to_map.pose)

    def send_last_map_to_odom_transform(self):
        if not(hasattr(self, 'translation') and hasattr(self, 'rotation')):
            print("sup dude")
            return
        self.tf_broadcaster.sendTransform(self.translation,
                                          self.rotation,
                                          rospy.get_rostime(),
                                          'odom',
                                          'map')

if __name__ == '__main__':
    n = RobotLocalizer()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        n.send_last_map_to_odom_transform()
        r.sleep()
