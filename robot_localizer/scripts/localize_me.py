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


    def particle_cloud(self, xy_theta=None):
        '''xy_theta'''

        # Make some noise
        linear_noise = .5
        angular_noise = math.pi/2.0

        # Use odom to create particle cloud where once there was none
        if xy_theta == None:
            xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)

        # Make me a new particle cloud
        self.particle_cloud = []
    	for x in range(self.particles):
    		x = xy_theta[0] + (random_sample()*linear_noise-(linear_noise/2.0))
    		y = xy_theta[1] + (random_sample()*linear_noise-(linear_noise/2.0))
    		theta = xy_theta[2] + (random_sample()*angular_noise-(angular_noise/2.0))
    		self.particle_cloud.append(Particle(x, y, theta))

        # normalize particles because all weights were originall set to 1 on default
        self.normalize_particles()
        self.robot_position()

    def particle_to_pose(self):
        orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

    def particle_updater(self, old_pose, new_pose):
        '''Updates the particles position based on the robots change in x, y, and theta'''

    	x_diff = new_pose.x - old_pose.x
    	self.x = self.x + diff

    	y_diff = new_pose.y - old_pose.y
    	self.y = self.y + diff

    	theta_diff = new_pose.theta - old_pose.theta
    	self.theta = self.theta + diff

        self.normalize_particles()

    def publish_particles(self, msg):
        ''' publish my particles'''
        particles = []
        for particle in self.particle_cloud:
            particles.append(particle.particle_to_pose())
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),
                                            frame_id=self.map.frame),pose=particles))

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

    def normalize_particles(self):
        ''' make sure all weights add up to 1.0'''
        sum_w = sum(particle.w for particle in self.particle_cloud)
        for particle in self.particle_cloud:
            particle.w/=sum_w

    def resample_particles(self):
        ''' resamples particles according to new weights which are updated
            based on laser scan messages
        '''
        self.normalize_particles()

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

    def filter(self, particle_cloud):
		'''Filters out particles. Currently randomly gets rid of x number of particles'''
        for x in range(50):
            remove_point = random.randint(1,len(particle_cloud))
            particle_cloud.pop(remove_point)

        return particle_cloud





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

        self.odom_pos = None
        self.odom_ori = None
        self.odom_header = None

    def odom_callback(self, msg):
        self.odom_header = msg.header
        self.odom_pos = msg.pose.pose.position
        self.odom_ori = msg.pose.pose.orientation

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
		'''Represents the position of the robot as a x, y , yaw tuple'''

        pose = self.transform_helper.convert_translation_rotation_to_pose(self.odom_pos, self.odom_ori)
        return self.transform.helper.convert_pose_to_xy_and_theta(pose)


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
