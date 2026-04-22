from geometry_msgs.msg import Point
import numpy as np

def point_subtract(lhs: Point, rhs: Point) -> Point:
    return Point(
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z
    )

def point_add(lhs: Point, rhs: Point) -> Point:
    return Point(
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z
    )

def point2vector2D(point: Point) -> np.ndarray:
    return np.array([point.x, point.y])