import numpy as np
from geometry_msgs.msg import Point

class Waypoint:
    def __init__(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __iter__(self):
        return iter((self.x, self.y, self.yaw))

    def __getitem__(self, index):
        return (self.x, self.y, self.yaw)[index]

    def __repr__(self):
        return f"Waypoint(x={self.x}, y={self.y}, yaw={self.yaw})"
    
    def getPoint(self) -> Point:
        return Point(self.x, self.y, 0)
    
    def getYaw(self) -> float:
        return self.yaw

WAYPOINTS = [
    Waypoint(0.11, 0.11, 0),
    Waypoint(1.71, 0.52, np.pi/2),
    Waypoint(3.4, 1.1, -np.pi/2),
    Waypoint(3.3, 2.6, np.pi/2),
    Waypoint(5.1, 0.4, -np.pi/2)
]
