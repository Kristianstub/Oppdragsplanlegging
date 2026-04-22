from enum import Enum

SIMULATION_OBSTRACLES = [
    [1.2,  1.45, 0.2,  0.4], # Obstacle Left Island
    [2.5,  1.45, 0.4,  0.4], # Obstacle Right Island
    [1.55, 0.7,  0.5,  0.2], # Valve 0
    [3.16, 0.7,  0.5,  0.2], # Valve 1
    [3.56, 1.75, 0.5,  0.2], # Pump 1
    [3.3,  0.0,  1.91, 0.2], # Wall reduction Large
    [0,    1,    0.5,  0.2], # Wall reduction Small, on left side
]

REAL_OBSTRACLES = [
    [1.2,  1.75, 0.2,  0.4], # Obstacle Left Island
    [2.32,  1.75, 0.4,  0.4], # Obstacle Right Island
    [1.55, 0.7,  0.5,  0.2], # Valve 0
    [3.16, 0.7,  0.5,  0.2], # Valve 1
    [3.56, 1.75, 0.5,  0.2], # Pump 1
    [3.3,  0.0,  1.91, 0.2], # Wall reduction Large
    [0,    1,    0.5,  0.2], # Wall reduction Small, on left side
]

class ProgramEnvironment(Enum):
    real = 'real'
    simulation = 'simulation'

    def __repr__(self):
        return self.value

    def __str__(self):
        return self.__repr__()
