"""Geographic utility functions."""

from typing import Optional, Union

def degrees_to_dms(degrees: Optional[float]) -> str:
    """Convert decimal degrees to Degrees Minutes Seconds string.
    
    Args:
        degrees: Decimal degrees
        
    Returns:
        Formatted string in DMS format
    """
    if not isinstance(degrees, (float, int)) or degrees == 0.0:
        return "--"
    is_negative = degrees < 0
    degrees = abs(degrees)
    d = int(degrees)
    m = (degrees - d) * 60
    return f"{'-' if is_negative else ''}{d}° {m:.4f}'"

def knots_to_kph(knots: Optional[Union[float, str]]) -> float:
    """Convert speed from knots to km/h.
    
    Args:
        knots: Speed in knots
        
    Returns:
        Speed in kilometers per hour
    """
    try:
        return float(knots) * 1.852 if knots is not None else 0.0
    except (ValueError, TypeError):
        return 0.0