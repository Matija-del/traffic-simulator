from typing import List, Union
from .constants import CarIndex
import numpy as np

class Road:
    """Represents cars riding around a loop.

    Parameters
    ----------
    name : str
        Name of the track.
    radius : float
        The physical radius of the circular track (must be positive).
    number_of_cars : int
        The quantity of running vehicles (must be > 0).
    v_initial : float
        Initial velocity reference for cars/limits.

    Raises
    ------
    ValueError
        If radius or number_of_cars are invalid.
    """

    def __init__(
            self,
            name: str,
            radius: float,
            number_of_cars: int,
            v_initial: float,
    ) -> None:
        
        # Constructor Argument Validation
        if radius <= 0:
            raise ValueError("Radius must be a positive number.")
        if number_of_cars <= 0:
            raise ValueError("Number of cars must be a positive integer.")

        self.name: str = name
        self._radius: float = radius
        self.numc: int = number_of_cars

        self._cars: np.ndarray = self.initiate_cars(numc=number_of_cars, v_initial=v_initial)

    # ==========================================================================
    #     Properties and Setters
    # ==========================================================================
    @property
    def radius(self) -> float:
        """Get the track radius."""
        return self._radius

    @radius.setter
    def radius(self, new_radius: float) -> None:
        """Set a new track radius (validated)."""
        if new_radius <= 0:
            raise ValueError("Radius must be a positive number.")
        self._radius = new_radius

    @property
    def cars(self) -> np.ndarray:
        """Get the structural matrix representing all cars."""
        return self._cars

    @cars.setter
    def cars(self, new_cars_data: np.ndarray) -> None:
        """Partially update vehicle properties (positions, velocities)."""
        self._cars[[CarIndex.POSITIONS, CarIndex.VELOCITIES]] = new_cars_data

    # ==========================================================================
    #     Initialization methods
    # ==========================================================================
    def initiate_cars(self, numc: int, v_initial : float, dt: float = 0.1) -> np.ndarray:
        """Configure and populate the state-matrix of the vehicles.

        Parameters
        ----------
        numc : int
            Number of cars.
        v_0 : float
            Starting speed limit boundary.
        dt : float, optional
            Step time frame, by default 0.1.

        Returns
        -------
        np.ndarray
            A 2D array container monitoring all internal properties.
        """
        cars = np.zeros((7, numc))

        # Position mapping using constants
        cars[CarIndex.POSITIONS] = np.linspace(0, 2 * np.pi, numc, endpoint=False)
        cars[CarIndex.VELOCITIES] = np.full(numc, v_initial)
        cars[CarIndex.LOOK_AHEAD] = np.random.randint(0, numc, (numc))
        
        # Standard reaction time
        cars[CarIndex.TAOS] = np.ones((numc)) * 2
        cars[CarIndex.MIN_DISTANCE] = cars[CarIndex.VELOCITIES] * cars[CarIndex.TAOS] / self.radius

        cars[CarIndex.ALPHAS] = np.random.normal(loc=50, scale=0.1, size=numc) 
        cars[CarIndex.BETAS] = np.random.normal(loc=10, scale=0.01, size=numc) 
        
        return cars
        
    def update_s_min(self):
        """Updates minimum distance base on the current speed and reaction time."""
        self.cars[CarIndex.MIN_DISTANCE] = self.cars[CarIndex.VELOCITIES] * self.cars[CarIndex.TAOS] / self.radius
        
        
