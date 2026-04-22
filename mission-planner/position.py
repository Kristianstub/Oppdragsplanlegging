from geometry_msgs.msg import Point

class Position:
    def __init__(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw

    def __iter__(self):
        return iter((self.x, self.y, self.yaw))

    def __getitem__(self, index):
        return (self.x, self.y, self.yaw)[index]

    def __repr__(self):
        return f"Position(x={self.x}, y={self.y}, yaw={self.yaw})"
    
    def getPoint(self) -> Point:
        return Point(self.x, self.y, 0)
    
    def getYaw(self) -> float:
        return self.yaw