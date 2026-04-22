from geometry_msgs.msg import Point
from position import Position
from dataclasses import dataclass
from hybridAStarPathfinding import main_hybrid_a
from pointUtils import *
from typing import List

def vertex2Position(pos: List[float]) -> Position:
    return Position(pos[0], pos[1], pos[2])

def calulateRoute(startingPosition: Position, targetWaypoint: Position) -> List[Position]:
    path, fig, ax = main_hybrid_a(1, startingPosition, targetWaypoint, True, False, True, show_plot=False)

    pathPositions = [vertex2Position(vertex.pos) for vertex in path]

    return pathPositions
