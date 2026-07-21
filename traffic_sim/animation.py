from typing import List
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation
from .constants import SegmentTypes

class Animation:
    """
    Encapsulates traffic simulation data and animation logic.
    """

    def __init__(self, geometric_data: List[np.ndarray], lines: dict) -> None:
        [self.radius, self.num_c, self.len, self.dt] = geometric_data
        self.lines_data = lines
        self.steps = int(1000/ self.dt)
        
        self.history: List[np.ndarray] = []

        # Pre-define animation objects
        self.fig = None
        self.ax = None
        self.legend = pd.DataFrame.from_dict(data={key: values[:-1] for key, values in SegmentTypes.SEGMENTS.items()}, orient="index", columns=["Speed Factor", "Lenght", "Color"])
 
        self.animated_cars = {}

    def animate_cars(self) -> int:
        """Launches an interactive Matplotlib window showing current traffic flows."""
        num_c = self.num_c
        radius = self.radius
        
        print(self.legend)
        
        self.fig, self.ax = plt.subplots()
        
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-radius * 1.2, radius * 1.2)
        self.ax.set_ylim(-radius * 1.2, radius * 1.2)
        self.ax.set_title("Traffic Flow")        
        
        for line in self.lines_data.values():
            (start, stop, color) = line
            x = self.radius * np.cos(np.linspace(start, stop, 1000) / self.radius)
            y = self.radius * np.sin(np.linspace(start, stop, 1000) / self.radius)
            
            plt.plot(x, y, color=color)
        
        
            

        history_arr = np.array(self.history)
        if history_arr.size == 0:
            print("No history data to animate.")
            return 0

        self.animated_cars = {}
        for i in range(num_c):
            car_angles = history_arr[:, i]
            x_cor = radius * np.cos(car_angles)
            y_cor = radius * np.sin(car_angles)
            # Initialize scatter with a default color (will be updated immediately)
            scatter = self.ax.scatter([x_cor[0]], [y_cor[0]], s=60, edgecolors='black', linewidth=0.5)
            self.animated_cars[i] = (x_cor, y_cor, scatter)

        def animate(frame: int):
            # Update Car Positions and Colors
            for i in range(num_c):
                x, y, scatter = self.animated_cars[i]
                idx = frame % len(x)
                scatter.set_offsets([[x[idx], y[idx]]])
                current_pos = self.history[frame] if frame < len(self.history) else self.history[-1]
            car_artists = tuple(self.animated_cars[i][2] for i in range(num_c))
            return car_artists 
        
        num_frames = len(self.history)
        ani = animation.FuncAnimation(
            self.fig,
            animate,
            frames=num_frames,
            interval=5,
            blit=True,
        )

        plt.show()
        
        return 0 

    def add_history(self, position: np.ndarray) -> None:
        """Append vehicle positioning records to track frames history."""
        self.history.append(position)   
    
    
