#!/usr/bin/env python
''' Gretchen and Isaac's Robot Localizer '''

from __future__ import print_function, division
import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField

from helper_functions import TFHelper
from helper_functions import *

class Particle(object):
    def __init__(self,x=0.0,y=0.0,theta=0.0,weight=1.0):
        """ Construct a new Particle
            x: the x-coord of the hypothesis relative to the map frame
            y: the y-coord of the hypothesis relative ot the map frame
            theta: the yaw of the hypothesis relative to the map frame
            weight: the particle weight (the class does not ensure that particle weights are normalized """
        self.weight = weight
        self.theta = theta
        self.x = x
        self.y = y


    def particle_to_pose(self):
        ''' A helper function to convert a particle to a geometry_msgs/Pose message'''
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))


class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
            initialized: a flag to make sure initialization is complete before updating anything
            base_frame: robot base coord frame
            map_frame: map coord frame
            odom_frame: odom coord frame
            scan_topic: scan topic to get scan msgs from
            num_particles: num of particles to use
            linear_threshold: linear movement before a filter update
            angular_threshold: angular movement before a filter update
            max_dist: the maximum distance to an obstacle in the occupancy grid
            pose_listener: a sub for pose estimates
            particle_pub: a pub for the particle cloud
            laser_subscriber: a sub for scan data
            tf_listener: listener for coord transforms
            tf_broadcaster: broadcaster for coord transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: Robot pose based on last filter update [x,y,theta]
            map: the map
    """
    def __init__(self):
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('localizer')

        self.base_frame = "base_link"   # Robot base frame
        self.map_frame = "map"          # Map coord frame
        self.odom_frame = "odom"        # Odom coord frame
        self.scan_topic = "scan"        # Laser scan topic

        self.num_particles = 500          # # of particles to use

        self.linear_threshold = 0.1             # the amount of linear movement before performing an update
        self.angular_threshold = math.pi/10     # the amount of angular movement before performing an update

        self.max_dist = 2.0   # maximum penalty to assess in the likelihood field model

        # pubs and subs
        rospy.Subscriber("init_pose", PoseWithCovarianceStamped, self.pose_updater)
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.laserCallback)
        
        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        
        # enable listening for and broadcasting coord transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # Initialize particle cloud
        self.particle_cloud = []

        self.current_odom_xy_theta = []

        # grab the map from the map server
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

        # initializes the occupancyfield which contains the map
        self.occupancy_field = OccupancyField()
        print("initialized")
        self.initialized = True


    def robot_pose_updater(self):
        ''' Update the estimate of the robot's pose given the updated particles.'''
        # first make sure that the particle weights are normalized
        self.particle_normalizer()

        # Calculate avg particle position based on pose
    	mean_particle = Particle(0, 0, 0, 0)
        mean_particle_theta_x = 0
        mean_particle_theta_y = 0
        for particle in self.particle_cloud:
            mean_particle.x += particle.x * particle.weight
            mean_particle.y += particle.y * particle.weight

            # Using trig to calculate angle (@Paul I hate Trigonometry!)
            distance_vector = np.sqrt(np.square(particle.y)+ np.square(particle.x))
            mean_particle_theta_x += distance_vector * np.cos(particle.theta) * particle.weight
            mean_particle_theta_y += distance_vector * np.sin(particle.theta) * particle.weight

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
    	for particle in self.particle_cloud:
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
        self.particle_normalizer()

        # creates choices and probabilities lists, which are the particles and their respective weights
        choices = []
        probabilities = []
        num_samples = len(self.particle_cloud)
        for particle in self.particle_cloud:
            choices.append(particle)
            probabilities.append(particle.weight)

        # re-makes the particle cloud according to a random sample based on the probability distribution of the weights
        self.particle_cloud = self.draw_random_sample(choices, probabilities, num_samples)

    def laser_particle_updater(self, msg):
        '''Updates the particle weights in response to the scan contained in the msg'''

        # Find the total error for each particle based on 36 laser measurements taken from the Neato's actual position
        for particle in self.particle_cloud:
            error = []
            for theta in range(0,360,10):
                rad = np.radians(theta)
                err = self.occupancy_field.get_closest_obstacle_distance(particle.x + msg.ranges[theta] * np.cos(particle.theta + rad), particle.y + msg.ranges[theta] * np.sin(particle.theta + rad))
                if (math.isnan(err)):   # if the get_closest_obstacle_distance method finds that a point is out of bounds, then the particle can't never be it
                    particle.weight = 0
                    break
                error.append(err**5)     # each error is appended up to a power to make more likely particles have higher probability
            if (sum(error) == 0):     # if the particle is basically a perfect match, then we make the particle almost always enter the next iteration through resampling
                particle.weight = 1.0
            else:
                particle.weight = 1.0/sum(error)   # the errors are inverted such that large errors become small and small errors become large

    def draw_random_sample(self, choices, probabilities, n):
        ''' 
        Take a random sample....thanks @Paul
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
        '''
        # sets up an index list for the chosen particles, and makes bins for the probabilities
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]  # chooses the new particles based on the probabilities of the old ones
        samples = []
        for i in inds:
            samples.append(deepcopy(choices[int(i)]))   # makes the new particle cloud based on the chosen particles
        return samples

    def pose_updater(self, msg):
        ''' Restart particle filter based on updated pose '''
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.particle_cloud_init(xy_theta)
        self.fix_map_to_odom_transform(msg)

    def particle_cloud_init(self, xy_theta=None):
        ''' 
        Initialize the particle cloud
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                particle cloud around. If None, odometry will be used instead
        '''

        # levels of noise to introduce variability
        linear_noise = 1
        angular_noise = math.pi/2.0

        #  if doesn't exist, use odom
        if xy_theta == None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(self.odom_pose.pose)

        # make a new particle cloud and create a bunch of particles
        self.particle_cloud = []
    	for x in range(self.num_particles):
    		x = xy_theta[0] + (random_sample()*linear_noise-(linear_noise/2.0))
    		y = xy_theta[1] + (random_sample()*linear_noise-(linear_noise/2.0))
    		theta = xy_theta[2] + (random_sample()*angular_noise-(angular_noise/2.0))
    		self.particle_cloud.append(Particle(x, y, theta))

        # normalize particles (weights set to 1 on default)
        self.particle_normalizer()
        self.robot_pose_updater()

    def particle_normalizer(self):
        '''Make sure the particle weights sum to 1'''
    	weights_sum = sum(particle.weight for particle in self.particle_cloud)
        for particle in self.particle_cloud:
            particle.weight /= weights_sum

    def particle_publisher(self, msg):
        '''Publishes the resampled particles'''
        particles = []
        for particle in self.particle_cloud:
            particles.append(particle.particle_to_pose())
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map_frame),
                                  poses=particles))

    def runRobot(self, msg):
        '''Handling laser data to update our understanding of the robot based on laser and odometry'''
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame,msg.header.frame_id,msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            return

        if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
            # need to know how to transform between base and odometry frames
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
        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.particle_cloud_init()
            # cache the last odometric pose so we can only update our particle filter if we move more than self.linear_threshold or self.angular_threshold
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            self.fix_map_to_odom_transform(msg)
        elif (abs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.linear_threshold or
              abs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.linear_threshold or
              abs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.angular_threshold):
            # we have moved far enough to do an update!
            self.odom_particle_updater(msg)    # update based on odometry
            if self.laserCallback:
                last_projected_scan_timeshift = deepcopy(self.laserCallback)
                last_projected_scan_timeshift.header.stamp = msg.header.stamp
                self.scan_in_base_link = self.tf_listener.transformPointCloud("base_link", last_projected_scan_timeshift)

            self.laser_particle_updater(msg)   # update based on laser scan
            self.robot_pose_updater()                # update robot's pose
            self.particle_resampler()               # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(msg)     # update map to odom transform now that we have new particles
        self.particle_publisher(msg) # publish particles

    def fix_map_to_odom_transform(self, msg):
            """ This method constantly updates the offset of the map and
                odometry coordinate systems based on the latest results from
                the localizer """
            (translation, rotation) = \
                self.transform_helper.convert_pose_inverse_transform(self.robot_pose)
            p = PoseStamped(
                pose=self.transform_helper.convert_translation_rotation_to_pose(translation,
                                                            rotation),
                header=Header(stamp=msg.header.stamp, frame_id='base_link'))
            self.tf_listener.waitForTransform('base_link',
                                            'odom',
                                            msg.header.stamp,
                                            rospy.Duration(1.0))
            self.odom_to_map = self.tf_listener.transformPose('odom', p)
            (self.translation, self.rotation) = \
                self.transform_helper.convert_pose_inverse_transform(self.odom_to_map.pose)

    def send_last_map_to_odom_transform(self):
        if not(hasattr(self, 'translation') and hasattr(self, 'rotation')):
            return
        self.tf_broadcaster.sendTransform(self.translation,
                                          self.rotation,
                                          rospy.get_rostime(),
                                          'odom',
                                          'map')

if __name__ == '__main__':
    n = ParticleFilter()
    r = rospy.Rate(5)

    while not(rospy.is_shutdown()):
        n.send_last_map_to_odom_transform()
        r.sleep()