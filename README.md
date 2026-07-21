# Traffic Simulation Library (`traffic_sim`)

A modular micro-simulation framework designed to model, analyze, and animate multi-car vehicular dynamics on circular and multi-segment track topologies using **NumPy**, **Pandas**, and **Matplotlib**.

This library encapsulates physical kinematics, driver reaction times ($\\tau$), speed sensitivity parameters ($\\alpha, \\beta$), car-following acceleration, and sector speed compliance to study complex emergent traffic phenomena like phantom traffic jams and bottleneck slowdowns.

---

## Key Features

- **Flexible Track Architecture:** Standard circular loops, sectorized tracks, and dynamic linked-list road compositions (`Road`, `Parts`, `LinkedRoad`).
- **Physics Engine:** Euler numerical integration for velocity tracking, distance gap coupling, dynamic driver reaction times, and speed limit compliance.
- **Type-Safe Matrix Mapping:** Enumerated indices (`IntEnum`) mapping state matrices to eliminate magic numbers across car profiles and road segments.
- **Interactive Visualizations:** Matplotlib `FuncAnimation` wrappers rendering polar vehicle histories into 2D Cartesian space with color-coded segment indicators and speed legends.

---

## Project Structure

```text
traffic_simulation/
│
├── pyproject.toml         # Modern build and packaging configuration
├── README.md              # Technical documentation
├── tests                   
    ├── test.py            # Unit test suite for physics, track classes, and logic
    └── test_geometry.py   # Unit test suite for geometry
└── traffic_sim/
    ├── __init__.py        # Package orchestration module
    ├── animation.py       # Scatter-based dynamic animation renderer and legend display
    ├── constants.py       # IntEnum mappings for matrix dimensions & segment profiles
    ├── road.py            # Base Road environment class managing car states
    ├── parts_road.py      # Sectorized track environment extending Road
    ├── linked_road.py     # Dynamic linked-list segment road implementation
    ├── geometry.py        # Geometric functions related to circuit motion and design
    └── physics.py         # Physics logic, car-following formulas, and Euler integration
```

---

## Installation

Install the library locally in editable mode via `pip`:

```bash
pip install -e .[dev]
```

### Setup

Clone the repository:
```bash
git clone [https://github.com/Matija-del/traffic-simulation.git](https://github.com/Matija-del/traffic-simulation.git)
cd traffic-simulation
```

### Dependencies
- `numpy >= 1.21.0`
- `pandas >= 1.3.0`
- `matplotlib >= 3.5.0`

---

# Example Project
```text
import numpy as np
from traffic_sim import *

# 1. Initialize a circular environment
road_of_parts = LinkedRoad(
    name="Circuit de Monaco",
    number_of_cars=10,
    v_initial=20.0,
)

parts_data = [road_of_parts.radius, road_of_parts.numc, road_of_parts.lenght, 0.1]
lines_data = road_of_parts.lines_dict

animation = Animation(geometric_data=parts_data, lines=lines_data)

dt = animation.dt

print(f"v_0: {road_of_parts.v}")

# 2. Main Simulation Loop
for step in range(animation.steps):
    
    # Extract structural state slices using clean wrappers
    positions = road_of_parts.cars[0]  # POSITIONS
    velocities = road_of_parts.cars[1]  # VELOCITIES
    road_of_parts.update_s_min()
    
    # Calculate geometric relationships
    gaps = s_diffs(positions)
    sector_angles = road_of_parts.segments_data[0] / road_of_parts.radius
    sectors = calc_sector(positions, sector_angles)
    target_speeds = road_of_parts.segments_data[1][sectors]  # Active zone speed limits
    
    # Calculate velocity deltas
    v_deltas = v_diffs(velocities)
    sec_v_deltas = sec_v_diffs(velocities, target_speeds)

    # Pack weights and determine accelerations curves
    (alphas, betas) = (road_of_parts.cars[4], road_of_parts.cars[5])
    min_gaps = road_of_parts.cars[4]
    

    car_accelerations = calc_acc_car(
        v=velocities,
        s_diff=gaps,
        s_min=min_gaps,
        v_diff=v_deltas[:,0],
        alpha=alphas
    )
        
    sector_accelerations = calc_acc_sect(
        cars=[positions, velocities],
        all_sectors=road_of_parts.segments_data,
        sector_positions=sectors,
        beta=betas
    )
        
    accelerations = calc_acc(
        acc_sect=sector_accelerations,
        acc_car=car_accelerations,
        s_diff=gaps,
        s_min=min_gaps,
        dt=dt
    )

    # Integrate to update kinematics
    new_v = calc_v(velocities, accelerations, dt)
    new_s = calc_s(positions, velocities, accelerations, road_of_parts.radius, dt)

    # Re-assign states via property boundaries
    road_of_parts.cars[0] = new_s % (2 * np.pi)  # Keeps cars bound inside circular wrap
    road_of_parts.cars[1] = v_check(v=new_v, v_sect=target_speeds)  # Keep speed safe
    
    # Append state to track history frames
    animation.add_history(road_of_parts.cars[0].copy())

# 3. View the live-rendered animation interface
animation.animate_cars()
```
---

## Module Architecture & API Overview

### 1. `road.py` (`Road` Class)
The base track class managing vehicle properties and matrix states.

- **`radius` (Property):** Gets or dynamically modifies track radius with non-positive boundary validation.
- **`cars` (Property):** 2D array tracking 7 key parameters for each car (positions, velocities, look-aheads, safe distances, reaction times, $\\alpha, \\beta$).
- **`initiate_cars` / `update_s_min`:** Populates initial positions around the ring and dynamically updates safe braking distances based on current velocities and reaction time ($\\tau$).

### 2. `parts_road.py` (`Parts` Class)
Extends `Road` to partition the circular track into $N$ equal angular sectors with dedicated target speed limits.

- **`parts` (Property):** Matrix holding angular boundaries (`ZONES`) and target speeds (`MAX_SPEEDS`) for each sector.

### 3. `linked_road.py` (`LinkedRoad` & `RoadSegment` Classes)
Extends `Road` with a linked-list pattern (`RoadSegment`) to model modular road types (*road*, *road work*, *speed up*, *slow down*).

- **`add_segment(segment_type)`:** Appends linked segments to construct custom road maps.
- **`generate_linked_road()`:** Computes track length, updates geometry, and compiles segment boundaries and color indices for physics and animation.
- **`change_speed_limit(segment, new_speed)`:** Dynamically adjusts sector speed limits at runtime.

### 4. `constants.py`
Provides type-safe matrix index accessors using Python `IntEnum`:

- **`CarIndex`:** Maps `POSITIONS`, `VELOCITIES`, `LOOK_AHEAD`, `MIN_DISTANCE`, `ALPHAS`, `BETAS`, `TAOS`.
- **`PartIndex`:** Maps `ZONES`, `LENGHT`, `MAX_SPEEDS`.
- **`SegmentIndex` & `SegmentTypes`:** Dictates speed factors, lengths, color styling, and map icons for road segment presets.

### 5. `physics.py`
Contains instantaneous kinematic and acceleration calculation routines:

- **`calc_acc_car(...)`:** Computes acceleration from car-following gaps using non-linear distance weighting and safe distance thresholds ($s_{min}$).
- **`calc_acc_sect(...)`:** Computes target acceleration required to transition towards sector speed limits.
- **`calc_acc(...)`:** Blends car-following and sector speed accelerations dynamically based on proximity ratios.
- **`calc_v(...)` & `calc_s(...)`:** Executes numerical Euler integration to update vehicle velocities and angular coordinates around the track radius.
- **`v_check(...)`:** Clamps speeds between 0 and 110% of sector limits.

### 6. `animation.py` (`Animation` Class)
Transforms angular tracking matrices into polar coordinate animations.

- **`add_history(position)`:** Appends coordinate snapshots for playback.
- **`animate_cars()`:** Runs Matplotlib `FuncAnimation`, plotting vehicles along circular paths, rendering colored track segments, and outputting speed factor legends.

---

## Running Tests

Execute the unit test suite via `pytest`:

```bash
pytest
```
