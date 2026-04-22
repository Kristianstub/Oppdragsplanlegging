"""
Quick map visualizer — no A* run needed.
Shows obstacles and waypoints to verify against the Gazebo world.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Map bounds (matches main_hybrid_a: lx=5.21, ly=2.75)
LX = 5.21
LY = 2.75

# Obstacles from map_grid: [x, y, width, height]
OBS = [
    [1.2,  1.45, 0.2,  0.4],
    [2.5,  1.45, 0.4,  0.4],
    [1.55, 0.7,  0.5,  0.2],
    [3.16, 0.7,  0.5,  0.2],
    [3.56, 1.75, 0.5,  0.2],
    [3.3,  0.0,  1.91, 0.2],
]

# Waypoints: [x, y, heading]
WAYPOINTS = [
    [1.71, 0.40, 0],  # wp1 (start)
    [3.4,  1.20, 0],  # wp2
    [3.3,  2.50, 0],  # wp3
    [4.7,  0.50, 0],  # wp4
]

fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(0, LX)
ax.set_ylim(0, LY)
ax.set_aspect('equal')
ax.set_title('Map verification (obstacles + waypoints)')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')

# Draw map border
border = patches.Rectangle((0, 0), LX, LY, linewidth=2, edgecolor='black', facecolor='white')
ax.add_patch(border)

# Draw obstacles
for i, (x, y, w, h) in enumerate(OBS):
    rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
    ax.add_patch(rect)
    ax.text(x + w/2, y + h/2, str(i+1), ha='center', va='center', fontsize=8, color='white')

# Draw waypoints
colors = ['green'] + ['blue'] * (len(WAYPOINTS) - 2) + ['red']
for i, (x, y, _) in enumerate(WAYPOINTS):
    label = f'wp{i+1} ({x}, {y})'
    ax.plot(x, y, 'o', color=colors[i], markersize=8, zorder=5)
    ax.annotate(label, (x, y), textcoords='offset points', xytext=(6, 6), fontsize=8)

# Draw path between waypoints
for i in range(len(WAYPOINTS) - 1):
    x0, y0, _ = WAYPOINTS[i]
    x1, y1, _ = WAYPOINTS[i+1]
    ax.annotate('', xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='->', color='steelblue', lw=1.5))

ax.grid(True, linestyle='--', alpha=0.4)
plt.tight_layout()
plt.show()
