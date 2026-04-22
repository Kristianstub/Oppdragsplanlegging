"""
Diagnostic script — checks each leg before running full hybrid A*.
Prints: position safety, 2D A* reachability, direct Dubins shot.
"""
import sys
sys.path.insert(0, '.')

from math import pi, tan
from utils.environment import Environment
from utils.car import SimpleCar
from utils.grid import Grid
from utils.astar import Astar
from utils.dubins_path import DubinsPath

OBS = [
    [1.2,  1.45, 0.2,  0.4],
    [2.5,  1.45, 0.4,  0.4],
    [1.55, 0.7,  0.5,  0.2],
    [3.16, 0.7,  0.5,  0.2],
    [3.56, 1.75, 0.5,  0.2],
    [3.3,  0.0,  1.91, 0.2],
]

LEGS = [
    ([1.71, 0.40, 0], [3.4,  1.20, 0]),  # wp1 -> wp2
    ([3.4,  1.20, 0], [3.3,  2.50, 0]),  # wp2 -> wp3
    ([3.3,  2.50, 0], [4.7,  0.50, 0]),  # wp3 -> wp4
]

env = Environment(OBS, lx=5.21, ly=2.75)
grid = Grid(env)

print(f"Grid size: {grid.n} x {grid.m} cells ({grid.cell_size}m each)")
print(f"Map size:  {env.lx} x {env.ly} m")
print(f"Min turning radius: {0.3/tan(pi/5):.3f} m  (l=0.3, max_phi=pi/5)")
print()

for i, (start, end) in enumerate(LEGS):
    label = f"Leg {i+1}: {start} -> {end}"
    print("=" * 60)
    print(label)
    print("=" * 60)

    car = SimpleCar(env, start, end, l=0.3)
    dubins = DubinsPath(car)

    # 1. Position safety
    start_safe = car.is_pos_safe(start)
    end_safe   = car.is_pos_safe(end)
    print(f"  Start safe:  {start_safe}")
    print(f"  End safe:    {end_safe}")

    if not start_safe or not end_safe:
        print("  !! One of the positions is UNSAFE — inside obstacle or out of bounds")

    # 2. 2D A* reachability on the grid
    astar = Astar(grid, end[:2])           # backward A*: starts at goal cell
    cost = astar.search_path(start[:2])
    if cost is None:
        print(f"  2D A*:       UNREACHABLE — grid has no path between start and end cells")
    else:
        print(f"  2D A*:       reachable  (cost = {cost} cells)")

    # 3. Direct Dubins shot (best-case: already near goal with clear line-of-sight)
    solutions = dubins.find_tangents(start, end)
    _, length, valid = dubins.best_tangent(solutions)
    print(f"  Direct Dubins shot valid: {valid}" + (f"  (length={length:.3f}m)" if valid else ""))

    if not valid:
        print(f"  Dubins solutions found: {len(solutions)}")
        for s in solutions:
            safe_straight = dubins.is_straight_route_safe(s.t1, s.t2)
            safe_arc1 = dubins.is_turning_route_safe(start, s.t1, s.d[0], s.c1, dubins.r)
            safe_arc2 = dubins.is_turning_route_safe(s.t2, end,   s.d[1], s.c2, dubins.r)
            print(f"    d={s.d} len={s.len:.3f}  arc1={safe_arc1}  straight={safe_straight}  arc2={safe_arc2}")

    print()
