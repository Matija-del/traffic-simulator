from enum import IntEnum

class CarIndex(IntEnum):
    """Indices mapping properties within the cars data matrix."""
    POSITIONS = 0
    VELOCITIES = 1
    ACCELERATION = 2
    LOOK_AHEAD = 3
    MIN_DISTANCE = 4
    CAR_WEIGHTS = 5
    SECTOR_WEIGHTS = 6

class PartIndex(IntEnum):
    """Indices mapping properties within the parts data matrix."""
    ZONES = 0
    MAX_SPEEDS = 1