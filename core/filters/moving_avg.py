"""Moving average filter implementation for position smoothing."""

from typing import List, Tuple
from core.filters.base import PositionFilter

class MovingAverageFilter(PositionFilter):
    """Moving average filter for position smoothing."""
    
    def __init__(self, max_history_size: int = 10):
        """Initialize with history buffer size.
        
        Args:
            max_history_size: Maximum number of positions to keep in history
        """
        super().__init__()
        self.max_history_size = max_history_size
        self.position_history: List[Tuple[float, float, float]] = []
        
    def add_position(self, lat: float, lon: float, alt: float):
        """Add a position to the history buffer.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        if lat == 0.0 or lon == 0.0:
            return  # Don't add invalid points
            
        self.position_history.append((lat, lon, alt))
        # Maintain history size limit
        if len(self.position_history) > self.max_history_size:
            self.position_history.pop(0)
            
    def get_filtered_position(self) -> Tuple[float, float, float]:
        """Calculate the moving average of positions.
        
        Returns:
            Tuple of (latitude, longitude, altitude)
        """
        if not self.position_history:
            return 0.0, 0.0, 0.0
            
        n = len(self.position_history)
        avg_lat = sum(pos[0] for pos in self.position_history) / n
        avg_lon = sum(pos[1] for pos in self.position_history) / n
        avg_alt = sum(pos[2] for pos in self.position_history) / n
        return avg_lat, avg_lon, avg_alt
        
    def reset(self):
        """Clear the position history."""
        self.position_history = []
        
    def set_params(self, **kwargs):
        """Set filter parameters.
        
        Args:
            max_history_size: Maximum number of positions to keep in history
        """
        if 'max_history_size' in kwargs:
            self.max_history_size = max(1, int(kwargs['max_history_size']))
            # Trim history if needed
            while len(self.position_history) > self.max_history_size:
                self.position_history.pop(0)