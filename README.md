# Traffic Simulation Library (`traffic_sim`)

A highly optimized, production-ready micro-simulation framework designed to model, analyze, and animate multi-car vehicular dynamics on circular, multi-sector track topologies using NumPy and Matplotlib.

This library encapsulates physical kinematics, driver look-ahead horizons, spatial tracking, and human factor behavioral delays to provide clean mathematical models for complex emergent traffic phenomena like shockwaves and phantom jams.

---

## Key Features

- **Geometric Modeling:** Advanced angular tracking, circular sector mapping, and adaptive spatial wrap-around gap analytics.
- **Physics Engine:** Modular Euler integration for velocity tracking, acceleration constraints, driver reaction horizons, and multi-factor resistance models.
- **Robust Architecture:** Complete typing implementation (`typing`), strict matrix validation, and clear data indexing structures using `IntEnum` to avoid magic constants.
- **Dynamic Visualization:** Built-in Matplotlib `FuncAnimation` wrappers to map polar matrix structures directly to Cartesian coordinates seamlessly.

---

## Project Structure

```text
traffic_simulation/
│
├── pyproject.toml            # Modern pip library packaging config
├── README.md                 # Project technical documentation
├── tests/
│   └── test_physics_geo.py   # Unit tests covering physics & geometry
└── traffic_sim/
    ├── __init__.py           # Package orchestration module
    ├── animation.py          # Scatter-based dynamic animation engine
    ├── constants.py          # IntEnum indexes mapping matrix structures
    ├── geometry.py           # Spatial tracking & angular functions
    ├── physics.py            # Kinematic formulas & integration
    └── road.py               # Principal Road environment class
```

---

## Installation

Install the library locally in editable mode directly via pip:

```bash
pip install -e .
```

### Dependencies
- `numpy >= 1.20.0`
- `matplotlib >= 3.4.0`

---

## Module Architecture & API Overview

### 1. `road.py` (`Road` Class)
The primary state orchestrator representing the circular ring track environment. It handles input safety schema checking via explicit property setters.

- **`radius` (Property):** Gets or dynamically modifies the track radius with safety boundary checks.
- **`cars` (Property):** Manages the core internal vehicular 2D matrix tracking parameters.
- **`initiate_cars` / `initiate_parts`:** Generates uniform linspaces for positioning, velocities, and sector bounds.

### 2. `constants.py`
Replaces unreadable index indexing matrices (`cars[0]`, `parts[1]`) with explicit, type-safe structures:
- `CarIndex`: Maps `POSITIONS`, `VELOCITIES`, `ACCELERATION`, `LOOK_AHEAD`, `MIN_DISTANCE`, `CAR_WEIGHTS`, `SECTOR_WEIGHTS`.
- `PartIndex`: Maps `ZONES`, `MAX_SPEEDS`.

### 3. `geometry.py`
Handles mathematical processing over polar/angular representations.
- `calc_look_ahead(look_ahead)`: Constructs a directional horizon factor array.
- `calc_sector(pos, nump)`: Decodes precisely which zone slice an arbitrary radian coordinate lies within.
- `s_diffs(positions)`: Resolves forward delta-space tracking safely across circular closures ($2\pi$).

### 4. `physics.py`
Encapsulates instantaneous system translation logic.
- `calc_acc(...)`: Computes multidimensional driver acceleration profiles including surrounding car interactions, speed limits, and friction forces.
- `calc_v(vel, acc, dt)` & `calc_s(pos, vel, acc, radius, dt)`: Standard numeric Euler integration frameworks.

### 5. `animation.py`
Transforms the polar structural histories directly into moving linear coordinate frames.
- `animate_cars(radius, history, num_c)`: Loops and redraws moving arrays cleanly over real-time axes.

---

## Quick Start Example

The example below sets up a circular road, computes positions across individual cycles, and animates the resulting track traffic:

```python
import numpy as np
from traffic_sim import Road, animate_cars, s_diffs, calc_sector, calc_acc, calc_v, calc_s, v_diffs, sec_v_diffs

# 1. Initialize a circular environment
road = Road(
    name="Circuit de Monaco", 
    radius=150.0, 
    number_of_parts=4, 
    number_of_cars=12, 
    v_initial=25.0, 
    reaction_factors=0.3
)

dt = 0.1
time_steps = 200

# 2. Main Simulation Loop
for step in range(time_steps):
    # Extract structural state slices using clean wrappers
    positions = road.cars[0] # POSITIONS
    velocities = road.cars[1] # VELOCITIES
    
    # Calculate geometric relationships
    gaps = s_diffs(positions)
    sectors = calc_sector(positions, road.num_p)
    target_speeds = road.parts[1][sectors] # Active zone speed limits
    
    # Calculate velocity deltas
    v_deltas = v_diffs(velocities)
    sec_v_deltas = sec_v_diffs(velocities, target_speeds)
    
    # Pack weights and determine acceleration curves
    weights = (road.cars[5], road.cars[6])
    look_ahead_mat = np.eye(road.num_c) # Identity placeholder for visualization
    min_gaps = road.cars[4]
    
    accelerations = calc_acc(
        s_diffs=gaps,
        v_diffs=v_deltas,
        sec_v_diffs=sec_v_deltas,
        weights=weights,
        look_ahead=look_ahead_mat,
        min_s_diffs=min_gaps,
        velocities=velocities,
        dt=dt
    )
    
    # Integrate to update kinematics
    new_v = calc_v(velocities, accelerations, dt)
    new_s = calc_s(positions, velocities, accelerations, road.radius, dt)
    
    # Re-assign states via property boundaries
    road.cars[0] = new_s % (2 * np.pi) # Keeps cars bound inside circular wrap
    road.cars[1] = np.clip(new_v, 0, 50) # Keep speed safe
    road.cars[2] = accelerations
    
    # Append state to track history frames
    road.add_history(road.cars[0].copy())

# 3. View the live-rendered animation interface
animate_cars(road.radius, road.history, road.num_c)
```

---

## 🧪 Testing

To verify the library mechanics, execute the comprehensive unit testing suite via terminal:

```bash
python -m unittest discover -s tests
```
