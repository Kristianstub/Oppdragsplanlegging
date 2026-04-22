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
    if len(path) == 0:
        return None
    if len(path) == 1:
        return path[0]

    lookahead_distance = max(0.0, lookahead_distance)
    seg_idx = find_closest_segment(path, robot)

    p1, p2 = path[seg_idx], path[seg_idx + 1]
    dx, dy = p2.x - p1.x, p2.y - p1.y
    seg_len2 = dx * dx + dy * dy

    if seg_len2 < 1e-9:
        proj_t = 0.0
    else:
        proj_t = ((robot.x - p1.x) * dx + (robot.y - p1.y) * dy) / seg_len2
        proj_t = float(np.clip(proj_t, 0.0, 1.0))

    current = np.array([p1.x + proj_t * dx, p1.y + proj_t * dy], dtype=float)
    remaining = lookahead_distance

    for i in range(seg_idx, len(path) - 1):
        end = np.array([path[i + 1].x, path[i + 1].y], dtype=float)
        v = end - current
        seg_len = float(np.linalg.norm(v))

        if seg_len < 1e-9:
            current = end
            continue

        if remaining <= seg_len:
            target = current + (remaining / seg_len) * v
            return Point(float(target[0]), float(target[1]), 0.0)

        remaining -= seg_len
        current = end

    # If the lookahead runs past the route, track the final waypoint.
    return path[-1]

def to_reference_frame(reference_position: Position, target: Point) -> Point:
    dx = target.x - reference_position.x
    dy = target.y - reference_position.y
    lx =  np.cos(reference_position.getYaw()) * dx + np.sin(reference_position.getYaw()) * dy
    ly = -np.sin(reference_position.getYaw()) * dx + np.cos(reference_position.getYaw()) * dy
    return Point(lx, ly, 0)

def pure_pursuit(robot_position: Position, lookahead_point: Point, lookahead_distance: float, speed: float) -> Twist:
    relative_point = to_reference_frame(robot_position, lookahead_point)
    geometric_ld = np.hypot(relative_point.x, relative_point.y)
    # Keep the denominator from collapsing near the target to avoid huge turn commands.
    ld = max(0.7 * lookahead_distance, geometric_ld, 1e-3)
    curvature = 2 * relative_point.y / (ld**2)
    heading_error = np.arctan2(relative_point.y, relative_point.x)

    ret = Twist()
    ret.linear.x = max(0.0, speed * np.cos(heading_error))
    ret.angular.z = curvature * speed

    return ret


def clamp_twist(twist: Twist, max_linear: float, max_angular: float) -> Twist:
    bounded = Twist()
    bounded.linear.x = float(np.clip(twist.linear.x, 0.0, max_linear))
    bounded.angular.z = float(np.clip(twist.angular.z, -max_angular, max_angular))
    return bounded


def adaptive_lookahead(base_lookahead: float, speed: float, heading_error: float,
                       min_lookahead: float, max_lookahead: float) -> float:
    # Increase lookahead slightly with speed and shrink it when heading error is large.
    speed_term = 0.5 * speed
    heading_term = max(0.25, np.cos(heading_error))
    ld = (base_lookahead + speed_term) * heading_term
    return float(np.clip(ld, min_lookahead, max_lookahead))