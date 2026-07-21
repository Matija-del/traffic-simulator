from typing import Optional, Tuple
import numpy as np

def sign(arr1, arr2):
    return np.where(arr1>arr2, 1, -1)
    
def reactor(arr1, arr2):
    return np.where(arr1>arr2, 1, -1) * np.e**((arr1 - arr2) / arr1)

def v_diffs(speeds: np.ndarray) -> np.ndarray:
    """Compute difference matrix of speeds between all cars.

    Parameters
    ----------
    speeds : np.ndarray
        1D array of individual vehicle speeds.

    Returns
    -------
    np.ndarray
        2D matrix containing delta speeds.
    """
    return np.array([speeds - speed for speed in speeds])


def sec_v_diffs(speeds: np.ndarray, comp_speeds: np.ndarray) -> np.ndarray:
    """Calculate speed differences relative to designated reference sector speeds.

    Parameters
    ----------
    speeds : np.ndarray
        Current speeds of the vehicles.
    comp_speeds : np.ndarray
        Reference speed limit of the sector each vehicle is currently in.

    Returns
    -------
    np.ndarray
        Differences between the target sector speed limit and actual speeds.
    """
    return comp_speeds - speeds

def calc_acc_car(
        v: np.ndarray,
        s_diff: np.ndarray,
        s_min: np.ndarray,
        v_diff: np.ndarray,    
        alpha: np.ndarray
) -> np.ndarray:
    """Calculate the vehicle's acceleration contribution based on car-following behavior.

    Parameters
    ----------
    v : np.ndarray
        Current speeds of the vehicles.
    s_min : np.ndarray
        Minimum safe distance threshold between vehicles.
    v_diff : np.ndarray
        Speed differences between leading and following vehicles (v_lead - v_following).
    alpha : np.ndarray
        Sensitivity coefficient for speed-to-distance coupling.

    Returns
    -------
    np.ndarray
        Acceleration component derived from interactions with leading vehicles.
    """
    
    safe_s = (s_diff - s_min)**4   
    
    acc = alpha * sign(s_diff, s_min) / safe_s

    return acc
    
def calc_acc_sect(cars: np.ndarray, all_sectors: np.ndarray, sector_positions: np.ndarray, beta: np.ndarray) -> np.ndarray:
    """Calculate the acceleration contribution based on sector speed limit compliance.

    Parameters
    ----------
    cars : np.ndarray
        Car data(s_car, v_car).
    sector : np.ndarray
        Sector data(s_sect, v_sect).
    sector_positions : np.ndarray
        Sector of cars.
    beta : np.ndarray
        Sensitivity coefficient for matching sector target speeds.

    Returns
    -------
    np.ndarray
        Acceleration component derived from the sector speed adjustment.
    """
    [s_sect, len_sect, v_sect] = all_sectors
    switched_v_sect = np.concatenate((v_sect[1:], np.array([v_sect[0]])), axis=None)
    
    [s_car, v_car] = cars
    coeff = ((s_sect[sector_positions] - s_car) / len_sect[sector_positions])**2
    
    v_ratio = (switched_v_sect[sector_positions] - v_sect[sector_positions]) / (v_sect[sector_positions] + switched_v_sect[sector_positions])
    next_coeff = 1 - coeff
    current_coeff = coeff
    
    acc = beta * (coeff * reactor(v_sect[sector_positions], v_car) + v_ratio * next_coeff * reactor(switched_v_sect[sector_positions], v_car))
    
    return acc
    
def calc_acc(
        acc_sect: np.ndarray,
        acc_car: np.ndarray,
        s_diff: np.ndarray,
        s_min: np.ndarray,
        dt: float = 0.1
) -> np.ndarray: 
    """Blend sector and car-following acceleration weights safely using dynamic proximity.

    Parameters
    ----------
    acc_sect : np.ndarray
        Acceleration component based on sector speed compliance.
    acc_car : np.ndarray
        Acceleration component based on car-following behavior.
    s_diff : np.ndarray
        Actual current distance gaps between vehicles.
    s_min : np.ndarray
        Minimum safe distance threshold between vehicles.
    dt : float, default 0.1
        Time step size for the model simulation.

    Returns
    -------
    np.ndarray
        Combined net acceleration for each vehicle.
    """

    safe_s_min = np.maximum(s_min, 1e-5)
     
 
    sect_weight = np.clip(s_diff / safe_s_min, 0, 1.0)
    car_weight = 1.0 - sect_weight
    
    acc = (sect_weight * acc_sect) + (car_weight * acc_car)
    
    return acc
     
def v_check(v: np.ndarray, v_sect: np.ndarray) -> np.ndarray:
    """Clamp vehicle speeds within realistic safe bounds.

    Parameters
    ----------
    v : np.ndarray
        Current or updated speeds of the vehicles.
    v_sect : np.ndarray
        Designated speed limits for the current sectors.

    Returns
    -------
    np.ndarray
        Speeds clipped between 0 (no reverse) and 110% of the sector limit.
    """
    return np.clip(v, 0, 1.1*v_sect)
    
def calc_v(vel: np.ndarray, acc: np.ndarray, dt: float = 0.1) -> np.ndarray:
    """Compute updated velocities using basic Euler integration.

    Parameters
    ----------
    vel : np.ndarray
        Current velocities.
    acc : np.ndarray
        Accelerations.
    dt : float, optional
        Time-step interval, by default 0.1.

    Returns
    -------
    np.ndarray
        Updated velocity states.
    """
    return vel + acc * dt
   
def calc_s(pos: np.ndarray, vel: np.ndarray, acc: np.ndarray, radius: float, dt: float = 0.1) -> np.ndarray:
    """Compute circular positions (radians) mapping trajectory steps.

    Parameters
    ----------
    pos : np.ndarray
        Current positions (radians).
    vel : np.ndarray
        Velocities.
    acc : np.ndarray
        Accelerations.
    radius : float
        Radius of the track.
    dt : float, optional
        Time-step interval, by default 0.1.

    Returns
    -------
    np.ndarray
        Updated polar coordinates on the track ring.
    """
    return pos + (vel * dt + 0.5 * acc * (dt ** 2)) / radius
