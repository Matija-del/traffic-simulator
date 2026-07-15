from typing import List
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


def animate_cars(radius: float, history: List[np.ndarray], num_c: int) -> int:
    """Launches an interactive Matplotlib window showing current traffic flows.

    Parameters
    ----------
    radius : float
        Circular highway radius.
    history : List[np.ndarray]
        List containing sequential frames tracking vehicles' coordinates.
    num_c : int
        Number of simulated cars.

    Returns
    -------
    int
        Status completion value (0 on normal exit).
    """
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(-radius * 1.2, radius * 1.2)
    ax.set_ylim(-radius * 1.2, radius * 1.2)

    # Convert circular track indices into analytical cartesian steps
    history_arr = np.array(history)  # (frames, num_c)
    animated_cars = {}

    for i in range(num_c):
        car_angles = history_arr[:, i]
        x_cor = radius * np.cos(car_angles)
        y_cor = radius * np.sin(car_angles)
        # Create a scatter element representing the physical vehicle on track
        scatter = ax.scatter(x_cor[0:1], y_cor[0:1])
        animated_cars[i] = (x_cor, y_cor, scatter)

    def animate(frame: int):
        for i in range(num_c):
            x, y, scatter = animated_cars[i]
            # Capture specific point matching frame keypoint
            X = x[frame: frame + 1]
            Y = y[frame: frame + 1]
            scatter.set_offsets(np.column_stack((X, Y)))
        return tuple(animated_cars[i][2] for i in range(num_c))

    # Initialize animation framework
    num_frames = len(history)
    ani = animation.FuncAnimation(
        fig,
        animate,
        frames=num_frames,
        interval=20,
        blit=True
    )

    plt.show()
    return 0