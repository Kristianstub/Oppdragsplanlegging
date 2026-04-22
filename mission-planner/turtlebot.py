import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import tf
from pid import PID
from position import Position
from pointUtils import *
from typing import List
from control import *

TURTLEBOT_START_POSITION = Point(0.2, 0.2, 0)
LOOKAHEAD_DISTANCE = 0.5
ANGLE_TARGET_THRESHOLD = 0.05
POSITION_TARGET_THRESHOLD = 0.05

class Turtlebot():
    """
    Turtlebot class, to move the turtlebot
    """

    position: Point
    yaw: float
    velocity: Twist

    def __init__(self):
        rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Turtlebot controller initiated.")
        rospy.on_shutdown(self.stop)

        self.position = TURTLEBOT_START_POSITION
        self.yaw = 0.0
        self.velocity = Twist()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # reading vehicle speed

        self.rate = rospy.Rate(10) # How often the controller loop is running [Hz]

        self.speed = 0.15
    
    def set_velocity(self, linear: float, angular: float):
        self.velocity.linear.x = linear
        self.velocity.angular.z = angular
        self.vel_pub.publish(self.velocity)

    def set_twist(self, twist: Twist):
        self.velocity = twist
        self.vel_pub.publish(self.velocity)
    
    def turn_around(self, targetYaw: float):
        yaw_pid_turn_on_point = PID(1, 0, 0)
        yaw_pid_turn_on_point.setReference(targetYaw)

        print("TURN AROUND")

        while not rospy.is_shutdown():
            angular = yaw_pid_turn_on_point.update(self.yaw)
            if abs(angular) < ANGLE_TARGET_THRESHOLD:
                break
            self.set_velocity(0, angular)
            self.rate.sleep()

        self.stop()

    def move_to_point(self, targetPoint: Point):
        position_error = point_subtract(targetPoint, self.position)

        # First turn to the righ direction
        target_yaw = np.arctan2(position_error.y, position_error.x)
        self.turn_around(target_yaw)

        print("MOVING LINEAR")

        # Second, drive to the target position.
        yaw_pid_moving = PID(1, 0.02, 0.2)
        yaw_pid_moving.setReference(target_yaw)

        # Move to the target point
        while not rospy.is_shutdown():
            position_error = point_subtract(self.position, targetPoint)
            position_error_vector = point2vector2D(position_error)
            distance_left = np.linalg.norm(position_error_vector)
            if distance_left < POSITION_TARGET_THRESHOLD:
                break

            angular = yaw_pid_moving.update(self.yaw)
            self.set_velocity(self.speed, angular)
            self.rate.sleep()

        self.stop()

    def move_to_position(self, position: Position):
        self.move_to_point(position.getPoint())
        self.turn_around(position.getYaw())
    
    def follow_route(self, route: List[Position]):

        self.turn_around(route[1].getYaw())

        path = [vertex.getPoint() for vertex in route]


        print("Using pure pursuit")
        while not rospy.is_shutdown():
            lookahead_point = find_lookahead_point(path, self.position, LOOKAHEAD_DISTANCE)
            if lookahead_point is None:
                break
            targetTwist = pure_pursuit(self.getPosition(), lookahead_point, LOOKAHEAD_DISTANCE, self.speed)
            self.set_twist(targetTwist)
            self.rate.sleep()
        
        # targetPoint = path[-1]
        # while not rospy.is_shutdown():
        #     position_error = point_subtract(self.position, targetPoint)
        #     position_error_vector = point2vector2D(position_error)
        #     distance_left = np.linalg.norm(position_error_vector)
        #     if distance_left < self.POSITION_TARGET_THRESHOLD:
        #         break

        print("Moving to the last Position")
        self.move_to_position(route[-1])

    def stop(self):
        self.velocity = Twist()
        self.vel_pub.publish(self.velocity)
        rospy.sleep(1)

    def odom_callback(self, msg: Odometry):
        # Get (x, y, yaw) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.yaw = yaw
        self.position = point_add(msg.pose.pose.position, TURTLEBOT_START_POSITION)
    
    def getPosition(self) -> Position:
        return Position(self.position.x, self.position.y, self.yaw) 
    
    def getYaw(self) -> float:
        return self.yaw
