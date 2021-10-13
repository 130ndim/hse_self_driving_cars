#!/usr/bin/python3

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Chase:
    def __init__(self):
        rospy.Subscriber('/turtle1/pose', Pose, self.chase)
        rospy.Subscriber('/turtle2/pose', Pose, self.get_position)
        self._pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        self._x = 0.
        self._y = 0.
        self._theta = 0.

    def chase(self, msg):
        dx, dy = msg.x - self._x, msg.y - self._y

        msg_chaser = Twist()
        msg_chaser.angular.z = math.atan2(dy, dx) - self._theta
        msg_chaser.linear.x = dx
        msg_chaser.linear.y = dy
        self._pub.publish(msg_chaser)

    def get_position(self, msg):
        self._x, self._y, self._theta = msg.x, msg.y, msg.theta


if __name__ == '__main__':
    rospy.init_node('chase')
    Chase()
    rospy.spin()
