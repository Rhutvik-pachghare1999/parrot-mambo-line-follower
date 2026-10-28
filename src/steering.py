# Steering Logic (Chart 2 - STEER)
# Author: Rhutvik

"""Steering logic - mirrors Simulink Stateflow Chart 2.

Takes region analysis results and outputs yaw command.
"""

from dataclasses import dataclass
from config import steering
from image_processing import RegionResult


@dataclass
class SteeringOutput:
    """Output from steering logic."""
    yaw_command: float      # Normalized [-1, 1]
    yaw_rate_cmd: float     # Actual yaw rate command (rad/s)
    line_detected: bool     # Whether any line was detected


class SteeringController:
    """Chart 2: STEER logic with proportional yaw control."""
    
    def __init__(self):
        self.prev_error = 0.0
        
    def compute(self, region_result: RegionResult, dt: float = 0.033) -> SteeringOutput:
        """
        Compute steering command from region analysis.
        
        For DOWNWARD-FACING camera (this simulation):
        - Left region of image  -> Path on drone's RIGHT -> Turn RIGHT (negative yaw)
        - Right region of image -> Path on drone's LEFT  -> Turn LEFT  (positive yaw)
        - Center region         -> Straight
        - None                  -> Land trigger
        
        This is INVERTED from forward-facing camera logic.
        """
        # Basic discrete logic (matches original but inverted for downward camera)
        if not (region_result.left_active or region_result.center_active or region_result.right_active):
            return SteeringOutput(
                yaw_command=0.0,
                yaw_rate_cmd=0.0,
                line_detected=False
            )
        
        # Determine which region has the line
        # Weight by pixel count for smoother transitions
        left_weight = region_result.left_pixels
        center_weight = region_result.center_pixels
        right_weight = region_result.right_pixels
        
        total = left_weight + center_weight + right_weight
        
        if total == 0:
            # Fallback to boolean logic (INVERTED for downward camera)
            if region_result.left_active:
                discrete_cmd = -1   # Left region -> path on right -> turn RIGHT
            elif region_result.right_active:
                discrete_cmd = 1    # Right region -> path on left -> turn LEFT
            else:
                discrete_cmd = 0
        else:
            # Weighted position: -1 (turn right) to +1 (turn left)
            # Right region (path on left) -> positive -> turn LEFT
            # Left region (path on right) -> negative -> turn RIGHT
            discrete_cmd = (right_weight - left_weight) / total
        
        # Apply deadband
        if abs(discrete_cmd) < steering.deadband:
            discrete_cmd = 0.0
        
        # Clamp
        discrete_cmd = np.clip(discrete_cmd, -steering.max_yaw_command, steering.max_yaw_command)
        
        # Convert to yaw rate command
        yaw_rate_cmd = discrete_cmd * steering.yaw_gain
        
        return SteeringOutput(
            yaw_command=discrete_cmd,
            yaw_rate_cmd=yaw_rate_cmd,
            line_detected=True
        )


import numpy as np