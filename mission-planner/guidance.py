from position import Position
from dataclasses import dataclass, field
from hybridAStarPathfinding import main_hybrid_a
from pointUtils import *
from typing import List
import matplotlib.pyplot as plt
from utils import car

def vertex2Position(pos: List[float]) -> Position:
    return Position(pos[0], pos[1], pos[2])

@dataclass
class PathSegment:
    direction: int
    vertices: List[Position] = field(default_factory=list)

def split_path_by_direction(path: List[car.State]) -> List[PathSegment]:
    """Split a flat car.State path into segments that each have a consistent direction."""
    if not path:
        return []

    segments: List[PathSegment] = []
    current_dir = path[0].direction
    current_segment: PathSegment = PathSegment(path[0].direction)

    current_segment.vertices.append(vertex2Position(path[0].pos))

    for i in range(1, len(path)):
        newVertex = path[i]

        if current_dir != newVertex.direction:
            segments.append(current_segment)
            current_dir = newVertex.direction
            current_segment = PathSegment(newVertex.direction)
        
        current_segment.vertices.append(vertex2Position(newVertex.pos))

    segments.append(current_segment)

    return segments

def calulateRoute(startingPosition: Position, targetWaypoint: Position, plot_route=False) -> List[PathSegment]:
    path, fig, ax = main_hybrid_a(
        1,
        startingPosition,
        targetWaypoint,
        reverse=True,
        extra=False,
        grid_on=True,
        animation=False,
        obstacle_safe_distance=0.1
    )

    if plot_route:
        plt.show(block=False)
        plt.pause(1)

    return split_path_by_direction(path)
