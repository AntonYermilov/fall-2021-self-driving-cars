#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from rospy import Publisher, Subscriber
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math


class Follower:
    def __init__(self):
        self.pub_slave = Publisher('/slave_turtle/cmd_vel', Twist, queue_size=10)
        self.sub_slave = Subscriber('/slave_turtle/pose', Pose, self.update)

        self.sub_simulator = Subscriber('/simulator/pose', Pose, self.follow)
        
        self.pose = Pose()
        self.eps = 1

    def update(self, my_pose):
        self.pose = my_pose

    def follow(self, pose):
        msg = Twist()

        dist = math.sqrt((pose.y - self.pose.y) ** 2 + (pose.x - self.pose.x) ** 2)

        if dist <= self.eps:
            return

        angle = math.atan2(pose.y - self.pose.y, pose.x - self.pose.x) - self.pose.theta + 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi

        msg.linear.x = min(dist, 1.0)
        msg.angular.z = angle
        
        self.pub_slave.publish(msg)


rospy.init_node('main')

f = Follower()

rospy.spin()
