# Parrot Mambo Line Follower - Configuration

"""Central configuration for the line follower simulation."""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class DroneConfig:
    """Physical drone parameters."""
    mass: float = 0.063  # kg (Parrot Mambo)
    max_velocity: float = 1.5  # m/s
    max_yaw_rate: float = 2.0  # rad/s
    max_vertical_speed: float = 1.0  # m/s
    hover_height: float = 1.0  # m
    takeoff_height: float = 0.5  # m
    landing_speed: float = 0.3  # m/s descent


@dataclass
class CameraConfig:
    """Simulated camera parameters."""
    width: int = 320
    height: int = 240
    fov_horizontal: float = 90.0  # degrees - wider for better curve tracking
    fov_vertical: float = 60.0   # degrees
    fps: int = 30
    # Y1UY2V format simulation (we'll work in RGB internally)
    format: str = "RGB"


@dataclass
class ImageProcessingConfig:
    """Color thresholding and region analysis parameters."""
    # HSV thresholds for colored line detection (red/orange tape)
    # OpenCV HSV: H in [0, 179], S in [0, 255], V in [0, 255]
    # Red wraps around 0/180, but our path renders as orange-ish due to anti-aliasing
    hsv_lower: Tuple[int, int, int] = (0, 40, 40)
    hsv_upper: Tuple[int, int, int] = (40, 255, 255)
    # Alternative red range (wraps around 180)
    hsv_lower2: Tuple[int, int, int] = (160, 50, 50)
    hsv_upper2: Tuple[int, int, int] = (180, 255, 255)
    
    # Region division: 3 horizontal bands (top third split into 3)
    region_top_frac: float = 0.33  # Top 33% of image
    region_left_frac: float = 0.33
    region_right_frac: float = 0.66
    
    # Minimum pixel count to consider region "active"
    min_pixels_threshold: int = 100
    
    # Morphological operations
    kernel_size: int = 5
    morph_iterations: int = 2


@dataclass
class StateMachineConfig:
    """State machine (Chart 1) parameters."""
    # Hover state
    hover_duration: float = 2.0  # seconds
    hover_tolerance: float = 0.1  # m
    
    # Cruise state
    cruise_velocity: float = 0.5  # m/s
    max_cross_track_error: float = 0.3  # m before rotation
    
    # Rotation state
    rotation_yaw_rate: float = 1.0  # rad/s
    rotation_tolerance: float = 0.1  # rad
    
    # Land state
    land_descent_rate: float = 0.3  # m/s
    land_trigger_lost_frames: int = 10  # frames with no line detected
    final_altitude: float = 0.05  # m


@dataclass
class SteeringConfig:
    """Steering logic (Chart 2) parameters."""
    yaw_gain: float = 1.5  # Proportional gain for yaw correction
    max_yaw_command: float = 1.0  # Normalized [-1, 1]
    deadband: float = 0.05  # Center deadband


@dataclass
class PathConfig:
    """Test path generation."""
    path_width: float = 0.15  # m (width of colored line)
    path_color: Tuple[int, int, int] = (255, 0, 0)  # Red in RGB
    background_color: Tuple[int, int, int] = (100, 100, 100)  # Gray floor


@dataclass
class SimConfig:
    """Simulation parameters."""
    dt: float = 0.033  # 30 Hz
    max_steps: int = 3000  # ~100 seconds
    record_video: bool = False
    video_fps: int = 30
    output_dir: str = "output"
    log_dir: str = "logs"


# Global config instances
drone = DroneConfig()
camera = CameraConfig()
image_proc = ImageProcessingConfig()
state_machine = StateMachineConfig()
steering = SteeringConfig()
path_cfg = PathConfig()
sim = SimConfig()