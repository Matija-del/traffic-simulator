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
    positions = road.cars[0]  # POSITIONS
    velocities = road.cars[1]  # VELOCITIES

    # Calculate geometric relationships
    gaps = s_diffs(positions)
    sectors = calc_sector(positions, road.num_p)
    target_speeds = road.parts[1][sectors]  # Active zone speed limits

    # Calculate velocity deltas
    v_deltas = v_diffs(velocities)
    sec_v_deltas = sec_v_diffs(velocities, target_speeds)

    # Pack weights and determine acceleration curves
    weights = (road.cars[5], road.cars[6])
    look_ahead_mat = np.eye(road.num_c)  # Identity placeholder for visualization
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
    road.cars[0] = new_s % (2 * np.pi)  # Keeps cars bound inside circular wrap
    road.cars[1] = np.clip(new_v, 0, 50)  # Keep speed safe
    road.cars[2] = accelerations

    # Append state to track history frames
    road.add_history(road.cars[0].copy())

# 3. View the live-rendered animation interface
animate_cars(road.radius, road.history, road.num_c)