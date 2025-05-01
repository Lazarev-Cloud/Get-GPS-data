"""Position filtering and smoothing algorithms."""

from core.filters.base import PositionFilter
from core.filters.moving_avg import MovingAverageFilter

try:
    from core.filters.kalman import KalmanFilter
    KALMAN_AVAILABLE = True
except ImportError:
    KALMAN_AVAILABLE = False