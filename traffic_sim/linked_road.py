from typing import List, Union
from .road import Road
from .constants import PartIndex, SegmentIndex, SegmentTypes, COUNTER
import numpy as np

class RoadSegment:
    """Represents a single part of the road."""
    def __init__(self, position: int, segment_type: str, v_initial: float=0) -> None: 
        self.position = position
        self._speed_limit = v_initial
        self.segment_type = segment_type
        data = SegmentTypes.SEGMENTS[segment_type]
        
        self.speed_factor = data[SegmentIndex.SPEED_FACTOR]
        self.lenght = data[SegmentIndex.LENGHT]
        self.icon = data[SegmentIndex.ICON]
        self.color = data[SegmentIndex.COLOR]
    
        self.next = None  # Links to the next segment
        
    # ================================================================
    #    Properties Functions
    # ================================================================
    @property
    def speed_limit(self) -> float:
        return self._speed_limit
    
    @speed_limit.setter
    def speed_limit(self, previous_speed: float) -> float:
        self._speed_limit = previous_speed * self.speed_factor
        

class LinkedRoad(Road):
    """Manages the linked list of road segments."""
    def __init__(
        self,
        name: str,
        number_of_cars: int,
        v_initial: float
    ) -> None:
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
            
        self._v = v_initial
        self.head = None  # Start of the road
        
        self.number_of_segments = 0
        self.road_map = []
        self.lenght = 0
        self.lines_dict = {}
        self.segments_data = self._get_segments()
        
        
        radius = self.lenght / (2*np.pi)
        
        super().__init__(
            name=name,
            radius=radius,
            number_of_cars=number_of_cars,
            v_initial=v_initial
        )
        
     # ================================================================
    #    Properties Functions
    # ================================================================   
    @property
    def v(self) -> float:
        """Returns current base speed of the Road."""
        return self._v
    
    @v.setter
    def v(self, new_speed: float) -> None:
        """Sets the new base sped of the road."""
        self._v = new_speed
        
    # ================================================================
    #    Node Creation Functions
    # ================================================================
    def add_segment(self, segment_type: str) -> None:
        """Adds a double node to the road. First node is always a road type segment and the other is by choice.
        
        Parameters
        __________
        segment_type: str
            Can be of values: road, road work, speed up, speed down.
            
        Raises
        ______
        TypeError
            If input is not of type str.
        ValueError
            If input is not of right value.
        """
        
        position = COUNTER + 1
        
        if type(segment_type) is not str:
            raise TypeError("Input variable segment_type must be of type str.")
            
        if segment_type not in SegmentTypes.SEGMENTS:
            raise ValueError("""Input variable segment_type must be one of "road", "road work", "speed up", "speed down".""")
        
        if not self.head:
            self.head = RoadSegment(position=position, segment_type="road", v_initial=self.v)
            self.head.next = RoadSegment(position=position+1, segment_type=segment_type, v_initial=self.v)
            self.number_of_segments += 2
            return
            
        current = self.head

        while current.next:
            position+=1
            current = current.next
            
            
        current.next = RoadSegment(position=position, segment_type="road", v_initial=self.v)  
        current.next.next = RoadSegment(position=position+1, segment_type=segment_type) 
        
        self.number_of_segments += 2
        return
        

    def generate_linked_road(self) -> np.ndarray:
        """Generates a linked road.
        
        Returns
        _______
        segments_data: np.ndarray
            Data for all the parts of shape (2, number_of_segments).
        """
        if self.number_of_segments==0:
            print("No existing nodes.")
            return
            
        counter = COUNTER
        
        segments_data = np.zeros((3, self.number_of_segments))
        
        lenght = 0        
        current = self.head
        
        while counter < self.number_of_segments:          
            current.speed_limit = self.v           
            self.road_map.append(f"{current.segment_type}: {current.icon}")
            
            lenght += current.lenght
            
            segments_data[PartIndex.LENGHT, counter] = current.lenght
            segments_data[PartIndex.ZONES, counter] = lenght
            segments_data[PartIndex.MAX_SPEEDS, counter] = current.speed_limit
            self.lines_dict[counter] = (lenght-current.lenght, lenght, current.color)
            
            
            current = current.next
            counter+=1
            
        self.lenght = lenght
        print(f"Road lenght: {lenght}")
        print("Linked Road: " + " -> ".join(self.road_map))
        
        return segments_data
        
    def _get_segments(self):
        '''
        decision = input("Choose a part(road, road work, speed up, slow down or click ENTER to exit): ")
        
        while decision!="":
            self.add_segment(decision)
            print(decision)
            
            decision = input("Choose a part(road, road work, speed up, slow down or click ENTER to exit): ")
        '''
        self.add_segment("road work")
        self.add_segment("road")
        self.add_segment("speed up")
        self.add_segment("slow down")
        return self.generate_linked_road()
    # ========================================================================================
    #    Active Functions
    # ========================================================================================        
    def change_speed_limit(self, segment : int, new_speed : float) -> None:
        """Change speed limit in one segment on the road."""
        
        if segment >= self.number_of_segments or segment < 0 or type(segment)!=int:
            raise ValueError(f"Value must be of type int between 0 and {self.number_of_segments}.")
        if type(new_speed)==str:
            raise TypeError("new_speed input must be of type int, double or float.")
        
        self.segments_data[PartIndex.MAX_SPEEDS][segment] = new_speed
        


