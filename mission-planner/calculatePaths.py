from hybridAStarPathfinding import main_hybrid_a
from pathlib import Path
import matplotlib.pyplot as plt
from waypoints import WAYPOINTS

def main():
    Path("figures").mkdir(parents=True, exist_ok=True)

    for i in range(2, max(len(WAYPOINTS), 5)):
        start_pos = WAYPOINTS[i - 1]
        end_pos = WAYPOINTS[i]
        _, fig, ax = main_hybrid_a(1, start_pos, end_pos, True, False, True, show_plot=False)
        fig.savefig(f"figures/WP{i - 1}-WP{i}.png")


if __name__ == "__main__":
    main()
