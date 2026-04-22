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
# TURTLEBOT_START_POSITION = Point(3.4, 1.2, -1.5707)
LOOKAHEAD_DISTANCE = 0.20  # Reduced from 0.35 for tighter corner following
MIN_LOOKAHEAD_DISTANCE = 0.12  # Reduced from 0.20 for better path adherence
MAX_LOOKAHEAD_DISTANCE = 0.35  # Reduced from 0.55

# === Speed Control Parameters ===
MAX_LINEAR_SPEED = 0.20
MIN_LINEAR_SPEED = 0.07
MAX_ANGULAR_SPEED = 1.00

# === Enhanced Feedback Gains (very aggressive corner-cutting prevention) ===
LATERAL_ERROR_P_GAIN = 2.0  # VERY AGGRESSIVE: strong lateral error correction (rad/s per meter)
HEADING_ERROR_SPEED_FACTOR = 0.8  # VERY AGGRESSIVE: rapid slowdown if heading is wrong
LATERAL_ERROR_SPEED_FACTOR = 1.5  # VERY AGGRESSIVE: strong slowdown when off-path
DISTANCE_FEEDBACK_SPEED_FACTOR = 0.6  # AGGRESSIVE: early slowdown near waypoints
PP_CLAMP_FACTOR = 0.4  # VERY AGGRESSIVE: 0.4 for sharp turns (was 0.7)
CURVATURE_SPEED_FACTOR = 2.5  # VERY AGGRESSIVE: 2.5 for tight curves (was 1.2)
# Keep positive for standard ROS yaw convention (+z = CCW).
ANGULAR_COMMAND_SIGN = 1.0
ALIGN_FINAL_HEADING = True  # Enforce final waypoint heading alignment
ANGLE_TARGET_THRESHOLD = 0.05
POSITION_TARGET_THRESHOLD = 0.05


def wrap_angle(angle: float) -> float:
    return float(np.arctan2(np.sin(angle), np.cos(angle)))

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
        self.odom_initialized = False
        self.odom_origin = Point(0.0, 0.0, 0.0)
        self.odom_origin_yaw = 0.0
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # reading vehicle speed

        self.rate = rospy.Rate(10) # How often the controller loop is running [Hz]

        self.speed = MAX_LINEAR_SPEED

    def wait_for_odometry(self, timeout_s: float = 2.0):
        start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and not self.odom_initialized:
            if rospy.Time.now().to_sec() - start > timeout_s:
                rospy.logwarn("Odometry not initialized yet; using last known pose.")
                break
            rospy.sleep(0.05)
    
    def set_velocity(self, linear: float, angular: float):
        self.velocity.linear.x = linear
        self.velocity.angular.z = ANGULAR_COMMAND_SIGN * angular
        self.vel_pub.publish(self.velocity)

    def set_twist(self, twist: Twist):
        self.velocity = twist
        self.velocity.angular.z = ANGULAR_COMMAND_SIGN * self.velocity.angular.z
        self.vel_pub.publish(self.velocity)
    
    def turn_around(self, targetYaw: float):
        self.wait_for_odometry()
        yaw_pid_turn_on_point = PID(0.6, 0, 0, use_ssa=True)
        yaw_pid_turn_on_point.setReference(targetYaw)

        print("TURN AROUND")

        while not rospy.is_shutdown():
            heading_error = np.arctan2(np.sin(targetYaw - self.yaw), np.cos(targetYaw - self.yaw))
            if abs(heading_error) < ANGLE_TARGET_THRESHOLD:
                break

            angular = yaw_pid_turn_on_point.update(self.yaw)
            self.set_velocity(0, float(np.clip(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)))
            self.rate.sleep()

        self.stop()

    def move_to_point(self, targetPoint: Point):
        self.wait_for_odometry()
        position_error = point_subtract(targetPoint, self.position)

        # First turn to the righ direction
        target_yaw = np.arctan2(position_error.y, position_error.x)
        self.turn_around(target_yaw)

        print("MOVING LINEAR")

        # Second, drive to the target position.
        yaw_pid_moving = PID(1, 0.02, 0.2, use_ssa=True)
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
        self.wait_for_odometry()

        path = [vertex.getPoint() for vertex in route]
        if not path:
            return

        max_lateral_error = 0.0
        min_speed_recorded = MAX_LINEAR_SPEED
        
        while not rospy.is_shutdown():
            endpoint = path[-1]
            dist_to_goal = np.hypot(endpoint.x - self.position.x, endpoint.y - self.position.y)
            if dist_to_goal < POSITION_TARGET_THRESHOLD:
                break

            segment_idx = find_closest_segment(path, self.position)
            segment_start = path[segment_idx]
            segment_end = path[min(segment_idx + 1, len(path) - 1)]
            segment_heading = np.arctan2(segment_end.y - segment_start.y, segment_end.x - segment_start.x)
            heading_error = np.arctan2(np.sin(segment_heading - self.yaw), np.cos(segment_heading - self.yaw))

            # === Compute path-following errors ===
            lateral_error = compute_lateral_error(path, self.position)
            path_heading = compute_path_heading(path, self.position)
            max_lateral_error = max(max_lateral_error, abs(lateral_error))

            # === Dynamic lookahead (adaptive based on heading) ===
            dynamic_lookahead = adaptive_lookahead(
                LOOKAHEAD_DISTANCE,
                self.speed,
                heading_error,
                MIN_LOOKAHEAD_DISTANCE,
                MAX_LOOKAHEAD_DISTANCE,
            )

            lookahead_point = find_lookahead_point(path, self.position, dynamic_lookahead)
            if lookahead_point is None:
                lookahead_point = endpoint

            # === Enhanced Pure Pursuit with lateral error feedback ===
            targetTwist = pure_pursuit_with_lateral_correction(
                self.getPosition(),
                lookahead_point,
                dynamic_lookahead,
                lateral_error,
                LATERAL_ERROR_P_GAIN,
                self.speed,
                pp_clamp_factor=PP_CLAMP_FACTOR
            )

            # === Multi-factor adaptive speed ===
            adaptive_speed = adaptive_speed_multi_factor(
                self.speed,
                heading_error,
                lateral_error,
                dist_to_goal,
                heading_factor=HEADING_ERROR_SPEED_FACTOR,
                lateral_factor=LATERAL_ERROR_SPEED_FACTOR,
                distance_factor=DISTANCE_FEEDBACK_SPEED_FACTOR,
                min_speed=MIN_LINEAR_SPEED
            )
            min_speed_recorded = min(min_speed_recorded, adaptive_speed)

            # === Curvature-based speed scaling (backup mechanism) ===
            curvature_mag = abs(targetTwist.angular.z) / max(1e-3, self.speed)
            speed_scale = 1.0 / (1.0 + CURVATURE_SPEED_FACTOR * curvature_mag)
            commanded_speed = max(MIN_LINEAR_SPEED, adaptive_speed * speed_scale)
            
            targetTwist.linear.x = min(targetTwist.linear.x, commanded_speed)
            targetTwist = clamp_twist(targetTwist, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)

            self.set_twist(targetTwist)
            self.rate.sleep()

        print(f"Route complete. Max lateral error: {max_lateral_error:.3f}m, Min speed: {min_speed_recorded:.3f}m/s")
        print("Moving to the last Position")
        self.stop()
        if ALIGN_FINAL_HEADING:
            self.turn_around(route[-1].getYaw())

    def stop(self):
        self.velocity = Twist()
        self.vel_pub.publish(self.velocity)
        rospy.sleep(1)

    def odom_callback(self, msg: Odometry):
        # Get (x, y, yaw) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        if not self.odom_initialized:
            self.odom_origin = msg.pose.pose.position
            self.odom_origin_yaw = yaw
            self.odom_initialized = True

        # Anchor odometry to planner start frame to avoid spawn-offset and heading drift confusion.
        self.position = Point(
            msg.pose.pose.position.x - self.odom_origin.x + TURTLEBOT_START_POSITION.x,
            msg.pose.pose.position.y - self.odom_origin.y + TURTLEBOT_START_POSITION.y,
            0.0,
        )
        self.yaw = wrap_angle(yaw - self.odom_origin_yaw + TURTLEBOT_START_POSITION.z)
    
    def getPosition(self) -> Position:
        return Position(self.position.x, self.position.y, self.yaw) 
    
    def getYaw(self) -> float:
        return self.yaw
