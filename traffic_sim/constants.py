from enum import IntEnum

class CarIndex(IntEnum):
    """Indices mapping properties within the cars data matrix."""
    POSITIONS = 0
    VELOCITIES = 1
    LOOK_AHEAD = 2
    MIN_DISTANCE = 3
    ALPHAS = 4
    BETAS = 5
    TAOS = 6

class PartIndex(IntEnum):
    """Indices mapping properties within the parts data matrix."""
    ZONES = 0
    MAX_SPEEDS = 1
