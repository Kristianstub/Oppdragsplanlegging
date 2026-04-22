from geometry_msgs.msg import Point
from position import Position
from dataclasses import dataclass
from hybridAStarPathfinding import main_hybrid_a
from pointUtils import *
from typing import List
import matplotlib.pyplot as plt

def vertex2Position(pos: List[float]) -> Position:
    return Position(pos[0], pos[1], pos[2])

def calulateRoute(startingPosition: Position, targetWaypoint: Position, plot_route=False) -> List[Position]:
    path, fig, ax = main_hybrid_a(1, startingPosition, targetWaypoint, False, False, True, animation=False)

    if plot_route:
        plt.show(block=False)
        plt.pause(1)

    pathPositions = [vertex2Position(vertex.pos) for vertex in path]

    return pathPositions
