#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi
import tf

WAYPOINTS = [[0.1, 0.1], [1.7, 0.7]]  # wp0 to wp1

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = 0
        self.Integrator = 0
        self.Integrator_max = 10
        self.Integrator_min = -10
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:
            self.error -= 2*pi
        elif self.error < -pi:
            self.error += 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator += self.error
        self.Integrator = max(min(self.Integrator, self.Integrator_max), self.Integrator_min)
        self.I_value = self.Integrator * self.Ki
        return self.P_value + self.I_value + self.D_value

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

class TurtlebotMove():
    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0, 0, 0)

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(10)

        for point in WAYPOINTS:
            self.move_to_point(point[0], point[1])
            rospy.sleep(1)
        self.stop()
        rospy.logwarn("Done! Robot reached all waypoints.")

    def move_to_point(self, x, y):
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector / sqrt(diff_x**2 + diff_y**2)
        theta = atan2(diff_y, diff_x)

        self.pid_theta.setPID(1, 0, 0)
        self.pid_theta.setPoint(theta)

        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()
        self.pid_theta.setPoint(theta)
        self.pid_theta.setPID(1, 0.02, 0.2)

        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector)
            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x = 1.5*linear
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def odom_callback(self, msg):
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        TurtlebotMove()
    except rospy.ROSInterruptException:
        pass