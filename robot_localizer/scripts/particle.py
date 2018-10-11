from __future__ import print_function, division
from helper_functions import TFHelper
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix

import tf

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
        return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], weight=orientation_tuple[3]))
