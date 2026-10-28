# State Machine (Chart 1 - Main FSM)
# Author: Rhutvik

"""Main Flight State Machine - simplified from Simulink Stateflow Chart 1.

States: Hover -> Cruise -> Land
Continuous steering handled during Cruise.
"""

from enum import Enum
from dataclasses import dataclass
from typing import Tuple
import numpy as np
from config import state_machine, drone


class FlightState(Enum):
    """Flight states matching original Chart 1."""
    HOVER = "HOVER"
    CRUISE = "CRUISE"
    LAND = "LAND"
    LANDED = "LANDED"


@dataclass
class StateOutputs:
    """Control outputs from state machine."""
    vx_body: float = 0.0      # Forward velocity command (m/s)
    vy_body: float = 0.0      # Lateral velocity command (m/s)
    vz_world: float = 0.0     # Vertical velocity command (m/s)
    yaw_rate_cmd: float = 0.0 # Yaw rate command (rad/s)


class StateMachine:
    """Main FSM controller (Chart 1)."""
    
    def __init__(self):
        self.state = FlightState.HOVER
        self.hover_timer = 0.0
        self.lost_line_frames = 0
        
    def update(self, dt: float, steering_yaw_rate: float, line_detected: bool, 
               drone_yaw: float, drone_z: float) -> Tuple[FlightState, StateOutputs]:
        """
        Update state machine and return control outputs.
        
        Args:
            dt: Timestep (s)
            steering_yaw_rate: Yaw rate command from steering controller (rad/s)
            line_detected: Whether any line region is active
            drone_yaw: Current yaw (rad)
            drone_z: Current altitude (m)
            
        Returns:
            (new_state, control_outputs)
        """
        outputs = StateOutputs()
        
        # State transitions
        if self.state == FlightState.HOVER:
            self.state, outputs = self._hover_state(dt, drone_z, outputs)
            
        elif self.state == FlightState.CRUISE:
            self.state, outputs = self._cruise_state(dt, steering_yaw_rate, line_detected, 
                                                      drone_yaw, drone_z, outputs)
            
        elif self.state == FlightState.LAND:
            self.state, outputs = self._land_state(dt, drone_z, outputs)
            
        elif self.state == FlightState.LANDED:
            outputs = StateOutputs()  # All zeros
        
        return self.state, outputs
    
    def _hover_state(self, dt: float, z: float, outputs: StateOutputs) -> Tuple[FlightState, StateOutputs]:
        """Hover: climb to hover height and stabilize."""
        self.hover_timer += dt
        
        target_h = drone.hover_height
        error = target_h - z
        
        # Simple P control for altitude
        outputs.vz_world = np.clip(error * 2.0, -drone.max_vertical_speed, drone.max_vertical_speed)
        
        # Transition to cruise after hover duration and at height
        if self.hover_timer >= state_machine.hover_duration and abs(error) < state_machine.hover_tolerance:
            return FlightState.CRUISE, outputs
        
        return FlightState.HOVER, outputs
    
    def _cruise_state(self, dt: float, steering_yaw_rate: float, line_detected: bool,
                      yaw: float, z: float, outputs: StateOutputs) -> Tuple[FlightState, StateOutputs]:
        """Cruise: follow line using continuous steering, transition to land if line lost."""
        
        # Check for line lost (land trigger)
        if not line_detected:
            self.lost_line_frames += 1
            if self.lost_line_frames >= state_machine.land_trigger_lost_frames:
                return FlightState.LAND, outputs
        else:
            self.lost_line_frames = 0
        
        # Forward velocity
        outputs.vx_body = state_machine.cruise_velocity
        outputs.vy_body = 0.0
        
        # Maintain hover height
        target_h = drone.hover_height
        outputs.vz_world = np.clip((target_h - z) * 1.0, -0.5, 0.5)
        
        # Continuous yaw control from steering
        outputs.yaw_rate_cmd = steering_yaw_rate
        
        return FlightState.CRUISE, outputs
    
    def _land_state(self, dt: float, z: float, outputs: StateOutputs) -> Tuple[FlightState, StateOutputs]:
        """Land: descend to ground."""
        outputs.vz_world = -state_machine.land_descent_rate
        outputs.vx_body = 0.0
        outputs.vy_body = 0.0
        outputs.yaw_rate_cmd = 0.0
        
        if z <= state_machine.final_altitude:
            return FlightState.LANDED, outputs
        
        return FlightState.LAND, outputs
    
    def reset(self):
        """Reset to initial state."""
        self.state = FlightState.HOVER
        self.hover_timer = 0.0
        self.lost_line_frames = 0