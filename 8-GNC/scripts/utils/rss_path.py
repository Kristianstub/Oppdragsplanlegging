from math import atan2, sqrt, pi


class RSSPath:
    """
    Rotate-Straight-Rotate path planner for differential drive robots.

    Optimal for robots with zero minimum turning radius (e.g. Turtlebot).
    Three phases:
      1. Rotate in place to face the goal.
      2. Drive straight to the goal position.
      3. Rotate in place to the goal heading.
    """

    def __init__(self, car):
        self.car = car

    def _compute_geometry(self, start_pos, end_pos):
        """ Compute path geometry between two poses. """

        x1, y1, theta1 = start_pos
        x2, y2, theta2 = end_pos

        dx = x2 - x1
        dy = y2 - y1
        dist = sqrt(dx**2 + dy**2)

        alpha = atan2(dy, dx) if dist > 1e-4 else theta1

        delta1 = (alpha - theta1 + pi) % (2*pi) - pi   # shortest rotation to face goal
        delta2 = (theta2 - alpha + pi) % (2*pi) - pi   # shortest rotation to goal heading

        return dist, alpha, delta1, delta2

    def path_cost(self, start_pos, end_pos):
        """
        Cost of the RSS path.
        Rotations are converted to a distance-equivalent using max_omega,
        so the cost is comparable to straight-line arc lengths.
        """

        dist, _, delta1, delta2 = self._compute_geometry(start_pos, end_pos)
        omega = self.car.max_omega

        return abs(delta1) / omega + dist + abs(delta2) / omega

    def is_path_safe(self, start_pos, end_pos, angle_step=0.1):
        """ Check that all three phases of the RSS path are obstacle-free. """

        x1, y1, theta1 = start_pos
        x2, y2, theta2 = end_pos

        dist, alpha, delta1, delta2 = self._compute_geometry(start_pos, end_pos)

        # Phase 1: rotation in place at start position
        n1 = max(int(abs(delta1) / angle_step), 1)
        for i in range(n1 + 1):
            theta = theta1 + delta1 * i / n1
            if not self.car.is_pos_safe([x1, y1, theta]):
                return False

        # Phase 2: straight segment — check swept rectangle
        if dist > 1e-4:
            vertex1 = self.car.get_car_bounding([x1, y1, alpha])
            vertex2 = self.car.get_car_bounding([x2, y2, alpha])
            swept = [vertex2[0], vertex2[1], vertex1[3], vertex1[2]]
            if not self.car.env.rectangle_safe(swept):
                return False

        # Phase 3: rotation in place at goal position
        n3 = max(int(abs(delta2) / angle_step), 1)
        for i in range(n3 + 1):
            theta = alpha + delta2 * i / n3
            if not self.car.is_pos_safe([x2, y2, theta]):
                return False

        return True

    def find_path(self, start_pos, end_pos):
        """ Validate and cost an RSS path. Returns (cost, valid). """

        if not self.is_path_safe(start_pos, end_pos):
            return None, False

        return self.path_cost(start_pos, end_pos), True

    def get_path(self, start_pos, end_pos, dt=1e-2):
        """ Generate the RSS path as a list of States. """

        x1, y1, theta1 = start_pos
        x2, y2, theta2 = end_pos

        dist, alpha, delta1, delta2 = self._compute_geometry(start_pos, end_pos)
        omega = self.car.max_omega

        path = []
        pos = [x1, y1, theta1]

        # Phase 1: rotate in place toward the goal
        n1 = max(int(abs(delta1) / (omega * dt)), 1)
        d_theta1 = delta1 / n1
        for _ in range(n1):
            path.append(self.car.get_car_state(pos))
            pos = [pos[0], pos[1], pos[2] + d_theta1]

        pos = [x1, y1, alpha]

        # Phase 2: drive straight to goal position
        if dist > 1e-4:
            n2 = max(int(dist / dt), 1)
            for _ in range(n2):
                path.append(self.car.get_car_state(pos))
                pos = self.car.step(pos, 0, 1, dt)

        pos = [x2, y2, alpha]

        # Phase 3: rotate in place to goal heading
        n3 = max(int(abs(delta2) / (omega * dt)), 1)
        d_theta2 = delta2 / n3
        for _ in range(n3):
            path.append(self.car.get_car_state(pos))
            pos = [pos[0], pos[1], pos[2] + d_theta2]

        path.append(self.car.get_car_state(end_pos))

        return path
