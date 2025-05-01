"""Kalman filter implementation for position smoothing."""

from typing import Tuple, Dict, Any, Optional
from core.filters.base import PositionFilter
from config.settings import NUMPY_AVAILABLE

if NUMPY_AVAILABLE:
    import numpy as np
else:
    # Define a dummy numpy module for type checking
    class DummyNumpy:
        def __getattr__(self, name):
            raise ImportError("NumPy is not available")
    np = DummyNumpy()

class KalmanFilter(PositionFilter):
    """Kalman filter for position smoothing."""
    
    def __init__(self):
        """Initialize the Kalman filter state."""
        super().__init__()
        self.initialized = False
        
        if not NUMPY_AVAILABLE:
            self.state = None
            return
            
        # Kalman filter state
        self.state = {
            'x': np.zeros(3),       # State vector [lat, lon, alt]
            'P': np.eye(3) * 500,   # Initial state covariance (high uncertainty)
            'F': np.eye(3),         # State transition matrix (identity for static position model)
            'H': np.eye(3),         # Measurement matrix
            'Q': np.eye(3) * 0.05,  # Process noise covariance (model uncertainty)
            'R': np.eye(3) * 10.0   # Measurement noise covariance (sensor uncertainty)
        }
        self.initialized = True
        
    def add_position(self, lat: float, lon: float, alt: float):
        """Add a position measurement to the filter.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        # If NumPy is not available or positions are invalid, do nothing
        if not NUMPY_AVAILABLE or not self.initialized:
            return
            
        # If input is invalid (0,0), don't update
        if lat == 0.0 or lon == 0.0:
            return
            
        # Apply Kalman filter step
        try:
            x = self.state['x']
            P = self.state['P']
            F = self.state['F']
            Q = self.state['Q']
            H = self.state['H']
            R = self.state['R']
            z = np.array([lat, lon, alt])  # Measurement

            # --- Predict ---
            x_pred = F @ x
            P_pred = F @ P @ F.T + Q

            # --- Update ---
            y = z - H @ x_pred             # Measurement residual
            S = H @ P_pred @ H.T + R       # Residual covariance
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                print("Kalman filter warning: Singular matrix S. Skipping update.")
                return

            K = P_pred @ H.T @ S_inv      # Kalman gain
            x_new = x_pred + K @ y
            P_new = (np.eye(len(x)) - K @ H) @ P_pred

            # Save updated state
            self.state['x'] = x_new
            self.state['P'] = P_new

        except np.linalg.LinAlgError:
            print("Kalman filter error: Singular matrix during calculation. Resetting filter.")
            self.reset()
        except Exception as e:
            print(f"Kalman filter error: {e}")
        
    def get_filtered_position(self) -> Tuple[float, float, float]:
        """Get the filtered position estimate.
        
        Returns:
            Tuple of (latitude, longitude, altitude)
        """
        if not NUMPY_AVAILABLE or not self.initialized:
            return 0.0, 0.0, 0.0
            
        # Return the current state estimate
        return float(self.state['x'][0]), float(self.state['x'][1]), float(self.state['x'][2])
        
    def reset(self):
        """Reset the filter state."""
        if not NUMPY_AVAILABLE:
            return
            
        # Reinitialize Kalman filter
        self.state = {
            'x': np.zeros(3),       # State vector [lat, lon, alt]
            'P': np.eye(3) * 500,   # Initial state covariance (high uncertainty)
            'F': np.eye(3),         # State transition matrix (identity for static position model)
            'H': np.eye(3),         # Measurement matrix
            'Q': np.eye(3) * 0.05,  # Process noise covariance (model uncertainty)
            'R': np.eye(3) * 10.0   # Measurement noise covariance (sensor uncertainty)
        }
        self.initialized = True
        
    def set_params(self, **kwargs):
        """Set filter parameters.
        
        Args:
            process_noise: Process noise scale factor (default: 0.05)
            measurement_noise: Measurement noise scale factor (default: 10.0)
        """
        if not NUMPY_AVAILABLE or not self.initialized:
            return
            
        if 'process_noise' in kwargs:
            q_scale = float(kwargs['process_noise'])
            self.state['Q'] = np.eye(3) * q_scale
            
        if 'measurement_noise' in kwargs:
            r_scale = float(kwargs['measurement_noise'])
            self.state['R'] = np.eye(3) * r_scale