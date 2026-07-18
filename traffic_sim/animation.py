from typing import List
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable
from .geometry import calc_sector


class Animation:
    """
    Encapsulates traffic simulation data and animation logic.
    Cars change color from Green (low congestion) to Red (high congestion).
    """

    def __init__(self, geometric_data: List[np.ndarray]) -> None:
        [self.radius, self.num_p, self.num_c, self.dt] = geometric_data
        
        self.steps = int(27 / self.dt)
        
        self.history: List[np.ndarray] = []
        self.congestion_list = np.zeros((self.num_c, self.steps))
        
        # 1. Define Green-to-Red Colormap
        # 0.0 = Green, 0.5 = White/Yellow, 1.0 = Red
        cdict = {
            'red':   ((0.0, 0.0, 0.0), (0.5, 1.0, 1.0), (1.0, 1.0, 1.0)),
            'green': ((0.0, 1.0, 1.0), (0.5, 1.0, 1.0), (1.0, 0.0, 0.0)),
            'blue':  ((0.0, 0.0, 0.0), (0.5, 1.0, 1.0), (1.0, 0.0, 0.0))
        }
        self.cmap = LinearSegmentedColormap('GreenRed', cdict)
        
        # Normalize congestion levels to [0, 1] for the colormap
        self.norm = Normalize(vmin=0, vmax=2)
        self.scalar_map = ScalarMappable(norm=self.norm, cmap=self.cmap)

        # Pre-define animation objects
        self.fig = None
        self.ax = None
        self.data_display = None
        self.animated_cars = {}

    def animate_cars(self) -> int:
        """Launches an interactive Matplotlib window showing current traffic flows."""
        num_c = self.num_c
        radius = self.radius
        
        self.fig, self.ax = plt.subplots()
        self.fig.subplots_adjust(right=0.7)
        
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-radius * 1.2, radius * 1.2)
        self.ax.set_ylim(-radius * 1.2, radius * 1.2)
        self.ax.set_title("Traffic Flow (Color = Congestion)")

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

        # Data Display Text
        self.data_display = self.fig.text(
            0.75, 0.95, 
            'Initializing...', 
            fontsize=9, 
            va='top', 
            family='monospace',
            bbox=dict(boxstyle="round,pad=0.5", fc="white", ec="gray", alpha=0.8)
        )

        def animate(frame: int):
            # Update Car Positions and Colors
            for i in range(num_c):
                x, y, scatter = self.animated_cars[i]
                idx = frame % len(x)
                scatter.set_offsets([[x[idx], y[idx]]])
                
                # 2. Determine Color based on Congestion
                # Find which sector the car is in at this frame
                # We re-calculate sector membership based on stored history positions
                current_pos = self.history[frame] if frame < len(self.history) else self.history[-1]
                
                if frame < len(self.history):
                    
                    # Get congestion level for that sector at this step
                    if frame < self.congestion_list.shape[1]:
                        level = self.congestion_list[i, frame]
                    else:
                        level = 0
                    
                    # Convert level to RGBA color
                    color = self.scalar_map.to_rgba(level)
                    scatter.set_facecolors([color])
                else:
                    scatter.set_facecolors(['gray'])

            # Update Data Display
            if frame < self.congestion_list.shape[1]:
                current_levels = self.congestion_list[:, frame]
                lines = [f"Frame: {frame}"]
                for i, level in enumerate(current_levels):
                    lines.append(f"Car {i+1}: Lvl {int(level)}")
                self.data_display.set_text("\n".join(lines))
            else:
                self.data_display.set_text(f"Frame: {frame}\n(Data ended)")

            car_artists = tuple(self.animated_cars[i][2] for i in range(num_c))
            return car_artists + (self.data_display,)
        
        num_frames = len(self.history)
        ani = animation.FuncAnimation(
            self.fig,
            animate,
            frames=num_frames,
            interval=1,
            blit=False,
            init_func=self._init_animation
        )

        plt.show()
        return 0
        
    def _init_animation(self):
        self.data_display.set_text('')
        for i in range(self.num_c):
            scatter = self.animated_cars[i][2]
            scatter.set_facecolors([self.scalar_map.to_rgba(0)])
        # Return None or empty tuple when blit=False
        return () 

    def note_congestion(self, step: int, s_diff: np.ndarray, s_min : np.ndarray) -> None:
        """Notes congestion levels of 0, 0.5, 1."""        
        level = np.zeros((self.num_c))
        
        for i in range(self.num_c):
            
            if s_diff[i] > s_min[i] * 2:
                level[i] = 2
            elif s_min[i] * 2 >= s_diff[i] > s_min[i]:
                level[i] = 1
            else:
                level[i] = 0
            
            self.congestion_list[i, step] = level[i]

    def add_history(self, position: np.ndarray) -> None:
        """Append vehicle positioning records to track frames history."""
        self.history.append(position)   
