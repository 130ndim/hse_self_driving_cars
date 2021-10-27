#!/usr/bin/env python

import numpy as np

import rospy
import scipy as sp
import scipy.signal

import sensor_msgs.point_cloud2 as pc2

from geometry_msgs.msg import Pose
from laser_geometry.laser_geometry import LaserProjection
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class ScanNode:
    _lp = LaserProjection()

    def __init__(self):
        self._scan_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.callback)
        self._scan_publisher = rospy.Publisher('/filtered_scan', LaserScan, queue_size=1)
        self._map_publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=1)

    def neighbor_filter(self, x, r, n_nbrs=8):
        assert n_nbrs % 2 == 0
        x_pad = np.zeros(len(x) + n_nbrs)
        n_half = n_nbrs // 2
        x_pad[:n_half] = x[-n_half:]
        x_pad[-n_half:] = x[:n_half]
        x_pad[n_half:-n_half] = x
        kernel = np.ones(n_nbrs + 1) / n_nbrs
        kernel[n_nbrs // 2] = 0.
        mask = abs(sp.signal.convolve(x_pad, kernel, mode='valid') - x) < r
        filtered = x * mask
        return filtered.tolist()

    def _filter(self, msg):
        ranges = np.array(msg.ranges)
        msg.ranges = self.neighbor_filter(ranges, 0.05)
        self._scan_publisher.publish(msg)

    def _draw_occupancy_grid(self, msg):
        pc = self._lp.projectLaser(msg)
        coords = np.array([[p[0], p[1]] for p in pc2.read_points(pc)])

        size = 20
        res = 0.5

        xs, ys = coords.T
        data, _, _ = np.histogram2d(xs, ys, bins=int(size // res))

        data = (data / data.max() * 255 - 128).clip(-128, 127).astype(np.int8)
        msg = OccupancyGrid(data=data.flatten().tolist())
        msg.info.height = data.shape[0]
        msg.info.width = data.shape[1]
        msg.info.origin.position.x = -size // 2
        msg.info.origin.position.y = -size // 2
        msg.info.resolution = res
        msg.header.frame_id = 'base_link'
        self._map_publisher.publish(msg)

    def callback(self, msg):
        self._filter(msg)
        self._draw_occupancy_grid(msg)


rospy.init_node('scan_node')

r = rospy.Rate(0.2)
r.sleep()

ScanNode()
rospy.spin()
