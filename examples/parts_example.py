import numpy as np
import matplotlib
matplotlib.use("qtagg")  #for Linux system if used
from matplotlib import animation
import matplotlib.pyplot
import pandas as pd
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

