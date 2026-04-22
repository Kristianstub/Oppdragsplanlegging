from typing import List
from geometry_msgs.msg import Point, Twist
import numpy as np
from position import Position

def find_closest_segment(path: List[Point], robot: Point) -> int:
    min_dist = float('inf')
    closest_idx = 0
    for i in range(len(path) - 1):
        p1, p2 = path[i], path[i+1]
        # project robot onto segment
        dx, dy = p2.x - p1.x, p2.y - p1.y
        denom = dx**2 + dy**2
        if denom < 1e-9:
            continue
        t = ((robot.x - p1.x)*dx + (robot.y - p1.y)*dy) / denom
        t = np.clip(t, 0, 1)
        proj = np.array([p1.x + t*dx, p1.y + t*dy])
        dist = np.linalg.norm(proj - np.array([robot.x, robot.y]))
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    return closest_idx

def find_lookahead_point(path: List[Point], robot: Point, lookahead_distance: float) -> Point:
    start = find_closest_segment(path, robot)

    for i in range(start, len(path) - 1):
        p1, p2 = path[i], path[i+1]
        dx, dy = p2.x - p1.x, p2.y - p1.y
        fx, fy = p1.x - robot.x, p1.y - robot.y

        a = dx**2 + dy**2
        b = 2 * (fx*dx + fy*dy)
        c = fx**2 + fy**2 - lookahead_distance**2

        disc = b**2 - 4*a*c
        if disc < 0:
            continue

        t = (-b + disc**0.5) / (2*a)  # take the larger t (forward along segment)
        if 0 <= t <= 1:
            return Point(p1.x + t*dx, p1.y + t*dy, 0)

    return None

def to_reference_frame(reference_position: Position, target: Point) -> Point:
    dx = target.x - reference_position.x
    dy = target.y - reference_position.y
    lx =  np.cos(reference_position.getYaw()) * dx + np.sin(reference_position.getYaw()) * dy
    ly = -np.sin(reference_position.getYaw()) * dx + np.cos(reference_position.getYaw()) * dy
    return Point(lx, ly, 0)

def pure_pursuit(robot_position: Position, lookahead_point: Point, lookahead_distance: float, speed: float) -> Twist:
    relative_point = to_reference_frame(robot_position, lookahead_point)
    
    curvature = 2 * relative_point.y / lookahead_distance**2

    ret = Twist()
    
    ret.linear.x = speed                  # constant forward speed
    ret.angular.z = curvature * speed     # omega = kappa * v
    
    return ret