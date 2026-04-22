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


def compute_lateral_error(path: List[Point], robot: Point) -> float:
    """
    Compute signed lateral (cross-track) error: distance from robot to path segment,
    positive if robot is left of path, negative if right.
    """
    if len(path) < 2:
        return 0.0
    
    seg_idx = find_closest_segment(path, robot)
    p1, p2 = path[seg_idx], path[seg_idx + 1]
    
    seg_dx = p2.x - p1.x
    seg_dy = p2.y - p1.y
    seg_len2 = seg_dx**2 + seg_dy**2
    
    if seg_len2 < 1e-9:
        return 0.0
    
    rob_dx = robot.x - p1.x
    rob_dy = robot.y - p1.y
    
    cross = seg_dx * rob_dy - seg_dy * rob_dx
    seg_len = np.sqrt(seg_len2)
    lateral_error = cross / seg_len
    
    return float(lateral_error)


def compute_path_heading(path: List[Point], robot: Point) -> float:
    """Compute heading (yaw) of path at closest segment."""
    if len(path) < 2:
        return 0.0
    
    seg_idx = find_closest_segment(path, robot)
    seg_idx = min(seg_idx, len(path) - 2)
    
    p1, p2 = path[seg_idx], path[seg_idx + 1]
    heading = np.arctan2(p2.y - p1.y, p2.x - p1.x)
    return float(heading)


def adaptive_speed_multi_factor(base_speed: float,
                                 heading_error: float,
                                 lateral_error: float,
                                 distance_to_goal: float,
                                 heading_factor: float = 0.5,
                                 lateral_factor: float = 1.0,
                                 distance_factor: float = 0.3,
                                 min_speed: float = 0.05) -> float:
    """
    Compute adaptive speed with AGGRESSIVE feedback from heading, lateral error, and distance.
    """
    # Heading feedback: aggressive reduction if heading error is large
    heading_scale = max(0.3, np.cos(heading_error))
    speed_heading = base_speed * (1.0 - heading_factor * (1.0 - heading_scale))
    
    # Lateral error feedback: exponential penalty for path deviation
    lateral_mag = abs(lateral_error)
    lateral_scale = np.exp(-lateral_factor * lateral_mag)
    speed_lateral = base_speed * lateral_scale
    
    # Distance feedback: aggressive slowdown within 0.5 m of goal
    goal_slowdown_range = 0.5
    if distance_to_goal < goal_slowdown_range:
        distance_scale = distance_to_goal / goal_slowdown_range
        speed_distance = base_speed * (1.0 - distance_factor * (1.0 - distance_scale))
    else:
        speed_distance = base_speed
    
    # Combine using geometric mean: all factors must be good
    combined = (speed_heading * speed_lateral * speed_distance) ** (1.0 / 3.0)
    adaptive_speed = min(combined, base_speed)
    
    return float(np.clip(adaptive_speed, min_speed, base_speed))


def pure_pursuit_with_lateral_correction(robot_position: Position,
                                          lookahead_point: Point,
                                          lookahead_distance: float,
                                          lateral_error: float,
                                          lateral_p_gain: float,
                                          speed: float,
                                          pp_clamp_factor: float = 0.7) -> Twist:
    """
    Enhanced pure pursuit with explicit lateral error P-control feedback.
    """
    relative_point = to_reference_frame(robot_position, lookahead_point)
    geometric_ld = np.hypot(relative_point.x, relative_point.y)
    
    ld = max(pp_clamp_factor * lookahead_distance, geometric_ld, 1e-3)
    curvature = 2 * relative_point.y / (ld**2)
    heading_error = np.arctan2(relative_point.y, relative_point.x)
    
    # Add lateral error feedback
    lateral_correction = lateral_p_gain * lateral_error
    
    ret = Twist()
    ret.linear.x = max(0.0, speed * np.cos(heading_error))
    ret.angular.z = curvature * speed + lateral_correction
    
    return ret