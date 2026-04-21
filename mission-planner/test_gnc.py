#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi
import tf

WAYPOINTS = [ [1.7, 0.7]]

class TurtlebotMove():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Waiting for simulated time...")
        while rospy.Time.now().to_sec() == 0:
            rospy.sleep(0.1)
        rospy.loginfo("Clock ready.")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)

        rospy.sleep(1.0)  # wait for first odom message

        for point in WAYPOINTS:
            rospy.loginfo(f"Moving to waypoint {point}")
            self.move_to_point(point[0], point[1])
            rospy.sleep(1)

        self.stop()
        rospy.logwarn("Done! Robot reached all waypoints.")

    def angle_diff(self, a, b):
        # Returns shortest angle from b to a
        d = a - b
        while d > pi:
            d -= 2 * pi
        while d < -pi:
            d += 2 * pi
        return d

    def move_to_point(self, x, y):
        diff_x = x - self.x
        diff_y = y - self.y
        dist = sqrt(diff_x**2 + diff_y**2)

        if dist < 0.05:
            rospy.loginfo("Already at target, skipping.")
            return

        target_angle = atan2(diff_y, diff_x)

        # Phase 1: rotate to face target
        rospy.loginfo(f"Phase 1: Rotating to face ({x}, {y})")
        while not rospy.is_shutdown():
            err = self.angle_diff(target_angle, self.theta)
            rospy.loginfo(f"  angle error: {err:.3f} rad")
            if abs(err) < 0.05:
                break
            angular = 0.5 * err
            angular = max(min(angular, 0.3), -0.3)
            self.vel.linear.x = 0.0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()

        # Phase 2: drive forward to target
        rospy.loginfo(f"Phase 2: Driving to ({x}, {y})")
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            dist = sqrt(diff_x**2 + diff_y**2)
            rospy.loginfo(f"  dist: {dist:.3f} m")
            if dist < 0.05:
                break
            target_angle = atan2(diff_y, diff_x)
            err = self.angle_diff(target_angle, self.theta)
            linear = min(0.2, dist)
            angular = 0.5 * err
            angular = max(min(angular, 0.3), -0.3)
            self.vel.linear.x = linear
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()

    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)
        rospy.sleep(0.5)

    def odom_callback(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(q)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        TurtlebotMove()
    except rospy.ROSInterruptException:
        pass
