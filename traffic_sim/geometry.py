import numpy as np

def calc_look_ahead(look_ahead: np.ndarray) -> np.ndarray:
    """Calculate the look-ahead matrix based on driver horizons.

    Parameters
    ----------
    look_ahead : np.ndarray
        Array containing integer values representing look-ahead steps for each driver.

    Returns
    -------
    np.ndarray
        A square matrix of look-ahead calculations.
    """
    num_c = len(look_ahead)
    calc_la = np.zeros((num_c, num_c))

    for i in range(num_c):
        n = int(look_ahead[i])
        if n > 0:
            calc_la[i, np.arange(n)] = 1.0 - np.arange(look_ahead[n]) / look_ahead[n]
        calc_la[i] = np.append(calc_la[i][-(i + 1):], calc_la[i][:-(i + 1)])

    return calc_la


def calc_sector(pos: np.ndarray, nump: int) -> np.ndarray:
    """Determine the sector index for each car based on its polar position.

    Parameters
    ----------
    pos : np.ndarray
        Array of circular positions (in radians).
    nump : int
        Total number of sector divisions/parts on the circular track.

    Returns
    -------
    np.ndarray
        Array of integer sector indices.
    """
    sectors = (pos // (2 * np.pi / nump)) % nump
    return sectors.astype(int)


def s_diffs(positions: np.ndarray) -> np.ndarray:
    """Calculate directional spatial differences (gaps) between successive cars.

    Parameters
    ----------
    positions : np.ndarray
        Sorted circular positions of cars.

    Returns
    -------
    np.ndarray
        Angular distances to the car ahead (accounting for circular wrap-around).
    """
    switched_positions = np.zeros_like(positions)
    switched_positions[0] = positions[-1]
    switched_positions[1:] = positions[:-1]

    diffs = switched_positions - positions
    diffs = np.where(diffs < 0, 2 * np.pi + diffs, diffs)

    return diffs
