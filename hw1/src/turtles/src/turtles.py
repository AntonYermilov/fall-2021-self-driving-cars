#! /usr/bin/python

import rospy
from rospy import Publisher, Subscriber

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

import math


class Follower:
    def __init__(self):
        self.pub_turtle2 = Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.sub_turtle2 = Subscriber('/turtle2/pose', Pose, self.update)
        self.sub_turtle1 = Subscriber('/turtle1/pose', Pose, self.follow)

        self.turtle2_pose = Pose()
        self.eps = 1

    def update(self, pose):
        self.turtle2_pose = pose

    def follow(self, pose):
        msg = Twist()

        dist = math.sqrt((pose.y - self.turtle2_pose.y) ** 2 + (pose.x - self.turtle2_pose.x) ** 2)

        if dist <= self.eps:
            return

        angle = math.atan2(pose.y - self.turtle2_pose.y, pose.x - self.turtle2_pose.x) - self.turtle2_pose.theta + 2 * math.pi
        while angle > math.pi:
            angle -= 2 * math.pi

        msg.linear.x = min(dist, 1.0)
        msg.angular.z = angle

        self.pub_turtle2.publish(msg)


rospy.init_node('main')
follower = Follower()
rospy.spin()
