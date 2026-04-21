from hybridAStarPathfinding import main_hybrid_a
import numpy as np
from pathlib import Path

WAYPOINTS = [
    [0.11, 0.11, 0],
    [1.71, 0.52, np.pi/2],
    [3.4, 1.1, -np.pi/2],
    [3.3, 2.6, np.pi/2],
    [5.1, 0.4, -np.pi/2]
]

def main():
    Path("figures").mkdir(parents=True, exist_ok=True)

    for i in range(2, len(WAYPOINTS)):
        start_pos = WAYPOINTS[i - 1]
        end_pos = WAYPOINTS[i]
        fig, ax = main_hybrid_a(1, start_pos, end_pos, True, False, True, show_plot=False)
        fig.savefig(f"figures/WP{i - 1}-WP{i}.png")


if __name__ == "__main__":
    main()
