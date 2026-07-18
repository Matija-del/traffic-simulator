from typing import Optional, Tuple
import numpy as np


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
        v : np.ndarray,
        s_diff : np.ndarray,
        s_min : np.ndarray,
        v_diff : np.ndarray,    
        alpha : np.ndarray,
        beta : np.ndarray,
        tao : np.ndarray
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
    beta : np.ndarray
        Sensitivity coefficient for responding to speed differences.
    tao : np.ndarray
        Reaction time delay parameter (time headway).

    Returns
    -------
    np.ndarray
        Acceleration component derived from interactions with leading vehicles.
    """
    # to the minimum gap, while reacting positively/negatively to the velocity difference.
    # When stopped (v=0, v_diff=0), acc becomes 0 instead of forcing a reverse.
    tao = np.maximum(tao, 1e-5)
    acc = -(alpha + beta) * v + alpha * (s_min - s_diff) / tao + beta * (v_diff)
    
    return acc
    
def calc_acc_sect(v : np.ndarray, v_sect : np.ndarray, beta : np.ndarray) -> np.ndarray:
    """Calculate the acceleration contribution based on sector speed limit compliance.

    Parameters
    ----------
    v : np.ndarray
        Current speeds of the vehicles.
    v_sect : np.ndarray
        Designated speed limits for the current sectors.
    beta : np.ndarray
        Sensitivity coefficient for matching sector target speeds.

    Returns
    -------
    np.ndarray
        Acceleration component derived from the sector speed adjustment.
    """
    acc = beta * (v_sect - v) / v_sect * v
    
    return acc
    
def calc_acc(
        acc_sect: np.ndarray,
        acc_car : np.ndarray,
        s_diff : np.ndarray,
        s_min : np.ndarray,
        dt : float = 0.1
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
    # Prevent division by zero errors safely
    safe_s_min = np.maximum(s_min, 1e-5)
     
    # If the car ahead is far away (s_diff >> s_min), car_weight drops to 0, 
    # allowing the sector speed model to take complete control.
    # If a vehicle gets dangerously close (s_diff -> 0), car_weight maximizes to 1.
    car_weight = np.clip(s_min / np.maximum(s_diff, 1e-5), 0, 1.0)
    sect_weight = 1.0 - car_weight
    
    acc = (sect_weight * acc_sect) + (car_weight * acc_car)
    
    return acc
     
def v_check(
        v : np.ndarray,
        v_sect : np.ndarray
) -> np.ndarray:
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
