# Drone Physics Simulation

"""Simple 3D drone dynamics simulation for Parrot Mambo."""

import numpy as np
from dataclasses import dataclass
from typing import Tuple
from config import drone, sim


@dataclass
class DroneState:
    """Complete drone state."""
    position: np.ndarray  # [x, y, z] in world frame (m)
    velocity: np.ndarray  # [vx, vy, vz] in world frame (m/s)
    yaw: float            # Heading angle (rad)
    yaw_rate: float       # Yaw rate (rad/s)
    armed: bool = False
    landed: bool = False


class DroneSimulator:
    """
    Simplified drone dynamics model.
    
    State: [x, y, z, vx, vy, vz, yaw, yaw_rate]
    Control: [vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd] in body frame
    """
    
    def __init__(self, initial_state: DroneState = None):
        if initial_state is None:
            self.state = DroneState(
                position=np.array([0.0, 0.0, 0.0]),
                velocity=np.array([0.0, 0.0, 0.0]),
                yaw=0.0,
                yaw_rate=0.0,
                armed=False,
                landed=True
            )
        else:
            self.state = initial_state
        
        self.dt = sim.dt
        self.max_vel = drone.max_velocity
        self.max_yaw_rate = drone.max_yaw_rate
        self.max_vz = drone.max_vertical_speed
        
        # Simple first-order response time constants
        self.tau_xy = 0.5  # Horizontal velocity response
        self.tau_z = 0.3   # Vertical velocity response
        self.tau_yaw = 0.2 # Yaw rate response
        
        self.log = []
    
    def step(self, vx_body: float, vy_body: float, vz_world: float, yaw_rate_cmd: float):
        """
        Advance simulation by one timestep.
        
        Args:
            vx_body: Forward velocity command in body frame (m/s)
            vy_body: Lateral velocity command in body frame (m/s)
            vz_world: Vertical velocity command in world frame (m/s)
            yaw_rate_cmd: Yaw rate command (rad/s)
        """
        if not self.state.armed or self.state.landed:
            return self._get_state_dict()
        
        # Convert body-frame velocity commands to world frame
        cos_yaw = np.cos(self.state.yaw)
        sin_yaw = np.sin(self.state.yaw)
        
        vx_world_cmd = cos_yaw * vx_body - sin_yaw * vy_body
        vy_world_cmd = sin_yaw * vx_body + cos_yaw * vy_body
        
        # First-order velocity response
        self.state.velocity[0] += (vx_world_cmd - self.state.velocity[0]) * self.dt / self.tau_xy
        self.state.velocity[1] += (vy_world_cmd - self.state.velocity[1]) * self.dt / self.tau_xy
        self.state.velocity[2] += (vz_world - self.state.velocity[2]) * self.dt / self.tau_z
        
        # Clamp velocities
        v_xy = np.linalg.norm(self.state.velocity[:2])
        if v_xy > self.max_vel:
            self.state.velocity[:2] *= self.max_vel / v_xy
        self.state.velocity[2] = np.clip(self.state.velocity[2], -self.max_vz, self.max_vz)
        
        # Yaw rate response
        self.state.yaw_rate += (yaw_rate_cmd - self.state.yaw_rate) * self.dt / self.tau_yaw
        self.state.yaw_rate = np.clip(self.state.yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)
        
        # Integrate position
        self.state.position += self.state.velocity * self.dt
        self.state.yaw += self.state.yaw_rate * self.dt
        
        # Ground collision
        if self.state.position[2] < 0.02:
            self.state.position[2] = 0.02
            self.state.velocity[2] = 0.0
            if abs(vz_world) < 0.1 and v_xy < 0.1:
                self.state.landed = True
        
        return self._get_state_dict()
    
    def takeoff(self, target_height: float = None):
        """Initiate takeoff to hover height."""
        if target_height is None:
            target_height = drone.takeoff_height
        self.state.armed = True
        self.state.landed = False
        # Takeoff handled by state machine sending vz commands
    
    def land(self):
        """Initiate landing."""
        self.state.armed = False
    
    def _get_state_dict(self) -> dict:
        """Return state as dictionary for logging."""
        return {
            'timestamp': len(self.log) * self.dt,
            'x': self.state.position[0],
            'y': self.state.position[1],
            'z': self.state.position[2],
            'vx': self.state.velocity[0],
            'vy': self.state.velocity[1],
            'vz': self.state.velocity[2],
            'yaw': self.state.yaw,
            'yaw_rate': self.state.yaw_rate,
            'armed': self.state.armed,
            'landed': self.state.landed
        }
    
    def log_state(self, fsm_state: str, steering_cmd: float, line_detected: bool):
        """Log current state with additional info."""
        entry = self._get_state_dict()
        entry.update({
            'fsm_state': fsm_state,
            'steering_cmd': steering_cmd,
            'line_detected': line_detected
        })
        self.log.append(entry)
    
    def save_log(self, filepath: str):
        """Save flight log to CSV."""
        import csv
        if not self.log:
            return
        keys = self.log[0].keys()
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.log)


def create_initial_state() -> DroneState:
    """Create initial drone state on ground at origin."""
    return DroneState(
        position=np.array([0.0, 0.0, 0.02]),
        velocity=np.array([0.0, 0.0, 0.0]),
        yaw=0.0,
        yaw_rate=0.0,
        armed=False,
        landed=True
    )