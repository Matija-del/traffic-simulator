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


def calc_acc(
        s_diffs: np.ndarray,
        v_diffs: np.ndarray,
        sec_v_diffs: np.ndarray,
        weights: Tuple[np.ndarray, np.ndarray],
        look_ahead: np.ndarray,
        min_s_diffs: np.ndarray,
        velocities: np.ndarray,
        r: float = 1.0,
        acc_lims: Optional[np.ndarray] = None,
        dt: float = 0.1
) -> np.ndarray:
    """Calculate the acceleration profiles for all vehicles.

    Parameters
    ----------
    s_diffs : np.ndarray
        Angular differences between adjacent cars.
    v_diffs : np.ndarray
        2D matrix of relative speed differences.
    sec_v_diffs : np.ndarray
        Speed differences relative to sector goals.
    weights : Tuple[np.ndarray, np.ndarray]
        A pair containing (car_weights, sector_weights).
    look_ahead : np.ndarray
        Look-ahead step matrix.
    min_s_diffs : np.ndarray
        Minimum safety angular gaps.
    velocities : np.ndarray
        Current speeds of the cars.
    r : float, optional
        Reaction sensitivity factor, by default 1.0.
    acc_lims : Optional[np.ndarray], optional
        Optional limits to clip calculated accelerations, by default None.
    dt : float, optional
        Time step delta in seconds, by default 0.1.

    Returns
    -------
    np.ndarray
        Calculated instantaneous accelerations.
    """
    car_weights, sector_weights = weights[0], weights[1]

    # Acceleration related to surrounding traffic interaction
    weighted_v_diffs = (v_diffs * look_ahead) @ car_weights.T
    c_acc = r * car_weights / (s_diffs - (min_s_diffs + weighted_v_diffs * dt))

    # Acceleration alignment to sector speed limits
    s_acc = sec_v_diffs * sector_weights

    # Air resistance and friction models
    mass = 1500.0
    r_acc = (0.18 * (velocities ** 2) + 150.0) / mass

    # Total acceleration merging factors
    tot_acc = c_acc + s_acc - r_acc

    if acc_lims is not None:
        tot_acc = np.clip(tot_acc, -acc_lims, acc_lims)

    return tot_acc


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