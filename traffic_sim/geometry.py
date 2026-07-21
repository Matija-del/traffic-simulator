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

def calc_sector(positions: np.ndarray, sector_positions: np.ndarray) -> np.ndarray:
    """Determine the sector index for each car based on its polar position.

    Parameters
    ----------
    pos : np.ndarray
        Array of positions.
    sector_positions : np.ndarray
        Array of positions.
    
    Returns
    -------
    np.ndarray
        Array of integer sector indices.
    """
    sector = [len(sector_positions[sector_positions <= car]) for car in positions]
    
    return np.array(sector).astype(int)

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
    switched_positions = np.concatenate((positions[1:], np.array([positions[0]])), axis=None)
    
    diffs = switched_positions - positions
    diffs = np.where(diffs < 0, 2 * np.pi + diffs, diffs)

    return diffs
