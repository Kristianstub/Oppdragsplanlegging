#!/usr/bin/env python3
import argparse
import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan
from os import system, name
import time
import re
import matplotlib.animation as animation
from datetime import datetime, timezone
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy
# Import here the packages used in your codes
from hybridAStarPathfinding import main_hybrid_a
from waypoints import WAYPOINTS
from position import Position
import subprocess
from pathlib import Path
from turtlebot import Turtlebot
from guidance import calulateRoute
from environment import ProgramEnvironment

""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 


# 1) Program here your AI planner 

def calculate_temporal_plan(cache):
    script_dir = Path(__file__).resolve().parent
    temporal_planning_dir = script_dir / '../temporal-planning-main'

    if cache:
        with open(temporal_planning_dir / 'plan', 'r') as file:
            return file.read()

    plan_script = temporal_planning_dir / 'calculatePlan.sh'
    result = subprocess.run([plan_script], capture_output=True, text=True)
    if result.stderr:
        print("Errors:", result.stderr)
        raise Exception("Could not calulate plan.")

    planStr = result.stdout
    return planStr

def parse_plan(plan_str):
    actions = []
    pattern = r'(\d+\.\d+):\s*\(\s*(.+?)\s*\)\s*\[(\d+\.\d+)\]'
    
    for match in re.finditer(pattern, plan_str):
        time = float(match.group(1))
        action_parts = match.group(2).split()
        duration = float(match.group(3))
        
        actions.append({
            'time': time,
            'action': action_parts[0],
            'args': action_parts[1:],
            'duration': duration
        })
    
    return actions

def calculate_plan(cache=False):
    print("Calculating plan...")

    plan_str = calculate_temporal_plan(cache)

    return parse_plan(plan_str)

#2) GNC module (path-followig and PID controller for the robot)
"""  Robot GNC module ----------------------------------------------------------------------
"""

# This is imported at the top

def move_turtlebot_to_position(turtlebot: Turtlebot, position: Position, plot_route: bool):
    print(f"Moving to {position}")
    turtlebotPosition = turtlebot.getPosition()
    route = calulateRoute(turtlebotPosition, position, plot_route)
    
    turtlebot.follow_route(route)

def move_turtlebot_to_waypoint(turtlebot: Turtlebot, args, plot_route: bool):
    waypointStr = args[2]
    waypointIndex = int(waypointStr[len("waypoint"):])
    waypoint = WAYPOINTS[waypointIndex]

    move_turtlebot_to_position(turtlebot, waypoint, plot_route)

# 3) Program here your path-finding algorithm
""" Hybrid A-star pathfinding --------------------------------------------------------------------
"""

# This is imported at the top.

#4) Program here the turtlebot actions (based in your AI planner)
"""
Turtlebot 3 actions-------------------------------------------------------------------------
"""

class TakePhoto:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False
        self.image = None

        # Connect image topic
        img_topic = "/camera/image"
        if os.getenv("ENVIRONMENT") == ProgramEnvironment.simulation.value:
            img_topic = "/camera/rgb/image_raw"

        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Wait for first image with timeout (up to 5 seconds)
        rospy.loginfo("Waiting for camera to start publishing...")
        start_time = rospy.get_time()
        timeout = 5.0
        while not self.image_received and (rospy.get_time() - start_time) < timeout:
            rospy.sleep(0.1)
        
        if self.image_received:
            rospy.loginfo("Camera image stream connected")
        else:
            rospy.logwarn("Camera not responding after 5 seconds")

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received and self.image is not None:
            # Save an image
            cv2.imwrite(img_title, self.image)
            rospy.loginfo(f"Picture saved: {img_title}")
            return True
        else:
            rospy.logwarn("No image available to save")
            return False
        



def Manipulate_OpenManipulator_x():
    print("Executing manipulate valve with OpenManipulator-X")

    import actionlib
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    arm_client     = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory',     FollowJointTrajectoryAction)
    gripper_client = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    rospy.loginfo("Waiting for arm and gripper action servers...")
    arm_client.wait_for_server(timeout=rospy.Duration(5.0))
    gripper_client.wait_for_server(timeout=rospy.Duration(5.0))

    def move_arm(positions, duration=2.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        arm_client.send_goal_and_wait(goal)

    def set_gripper(position, duration=1.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['gripper']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        gripper_client.send_goal_and_wait(goal)

    # 1) Move arm forward to reach the valve
    move_arm([0.0, -0.5, 0.3, 0.2], duration=2.0)

    # 2) Grip
    set_gripper(-0.01, duration=1.0)

    # 3) Ungrip
    set_gripper(0.01, duration=1.0)

    # 4) Move arm back to home position
    move_arm([0.0, -1.05, 0.35, 0.70], duration=2.0)

def making_turn_exe():
    print("Executing Make a turn")
    time.sleep(1)
    #Starts a new node
    #rospy.init_node('turtlebot_move', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    #speed = input("Input your speed (degrees/sec):")
    #angle = input("Type your distance (degrees):")
    #clockwise = input("Clockwise?: ") #True or false

    speed = 5
    angle = 180
    clockwise = True

    #Converting from angles to radians
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0   #should be from the odometer

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()

def check_pump_picture_ir_waypoint0():
    a=0
    while a<3:
        print("Taking IR picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

def check_seals_valve_picture_eo_waypoint0():
    a=0
    while a<3:
        print("Taking EO picture at waypoint0 ...")
        time.sleep(1)
        a=a+1
    time.sleep(5)

# Charging battery 
def charge_battery_waypoint0():
    print("chargin battery")
    time.sleep(5)


# Define the global varible: WAYPOINTS  Wpts=[[x_i,y_i]];
# global WAYPOINTS
# WAYPOINTS = [[1,1],[2,2]]
# These are included at the top




# 5) Program here the main commands of your mission planner code
""" Main code ---------------------------------------------------------------------------
"""
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('-env', type=ProgramEnvironment, choices=list(ProgramEnvironment), default=ProgramEnvironment.simulation, help="Simulation or real")  #A* heuristic
    p.add_argument('-p', action='store_true', help='Plot the planned paths')
    p.add_argument('-gripper', action='store_true', help='Gripper')
    args = p.parse_args()

    os.environ["ENVIRONMENT"] = f"{args.env}"

    try:
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print("AI planners: GraphPlan")
        print("Path-finding: Hybrid A-star")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 20.03.23")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        #input_t=input("")
        # 5.0) Testing the GNC module (uncomment lines to test)

        # aea
        # aea 2
        # aea 3
        # aea 4
        
	    # 5.1) Make a plan using temporal planner
        # 5.2) Reading the plan 
        plan_general = calculate_plan(cache=True)

        turtlebot = Turtlebot()

        # move_turtlebot_to_position(turtlebot, WAYPOINTS[2])

        # 5.3) Start mission execution 
        for step in plan_general:
            if step['action'] == 'move':
                print("Move turlebot", step['args'])
                move_turtlebot_to_waypoint(turtlebot, step['args'], args.p)
            elif step['action'] == 'take_picture':
                waypoint_name = step['args'][1] if len(step['args']) > 1 else 'unknown'
                img_title = f"figures/{waypoint_name}-{datetime.now(timezone.utc).isoformat()}.jpg"
                print(f"Take picture at {waypoint_name}")
                camera = TakePhoto()
                if camera.take_picture(img_title):
                    rospy.loginfo("Saved image " + img_title)
                else:
                    rospy.loginfo("No image received at " + waypoint_name)
            elif step['action'] == 'manipulate_valve':
                print("Manipulate valve")
                if args.gripper:
                    Manipulate_OpenManipulator_x()

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
