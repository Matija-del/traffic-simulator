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
    LENGHT = 1
    MAX_SPEEDS = 2
    
class SegmentIndex(IntEnum):
    """Segment data constants."""
    SPEED_FACTOR = 0
    LENGHT = 1
    ICON = 2
    COLOR = 3
    
class SegmentTypes:
    """Types of road parts"""
    SEGMENTS = {
        "road": (1, 500, "⚫ ", (0, 0, 0)),
        "road work": (0.5, 100, "🔴 ", (1, 0, 0)),
        "slow down": (0.75, 250, "🟠 ", (1, 165/255, 0)),
        "speed up": (1.25, 250, "🟢 ",	(0, 1, 0)),
    }
    
COUNTER = 0

