"""Base class for position filtering algorithms."""

from typing import Tuple


class PositionFilter:
    """Base class for position filtering/smoothing algorithms."""

    def __init__(self):
        """Initialize the filter."""
        pass

    def add_position(self, lat: float, lon: float, alt: float):
        """Add a position to the filter."""
        raise NotImplementedError("Subclasses must implement this method")

    def get_filtered_position(self) -> Tuple[float, float, float]:
        """Get the filtered position estimate.

        Returns:
            Tuple of (latitude, longitude, altitude)
        """
        raise NotImplementedError("Subclasses must implement this method")

    def reset(self):
        """Reset the filter state."""
        raise NotImplementedError("Subclasses must implement this method")

    def set_params(self, **kwargs):
        """Set filter parameters."""
        raise NotImplementedError("Subclasses must implement this method")
