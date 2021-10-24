#! /usr/bin/python3

import rospy
from rospy import Publisher, Subscriber, Rate

from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

import numpy as np


class ScanCallback:

    def __init__(self):
        self.marker = Publisher('/visualization_marker', Marker, queue_size=10)
        self.rate = Rate(20)

    def __call__(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        deltas = np.abs(ranges[2:] - 2 * ranges[1:-1] + ranges[:-2])

        # remove points with too big distances to their neighbours
        # as eps we use median value among all deltas
        eps = np.median(deltas) * 3

        mask = np.ones_like(ranges, dtype=bool)
        mask[1:-1] = deltas < eps

        angles = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))
        xs = ranges[mask] * np.cos(angles[mask])
        ys = ranges[mask] * np.sin(angles[mask])

        marker = Marker()

        marker.header.frame_id = 'base_link'
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = 0

        # geometry_msgs/Pose
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        # geometry_msgs/Vector3
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # std_msgs/ColorRGBA
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # geometry_msgs/Point[]
        marker.points = [Point(x, y, 0.0) for x, y in zip(xs, ys)]

        self.rate.sleep()
        self.marker.publish(marker)


rospy.init_node('main')
subscriber = Subscriber('/base_scan', LaserScan, ScanCallback())
rospy.spin()
