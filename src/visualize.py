# Visualization & Animation
# Author: Rhutvik

"""Matplotlib-based visualization for the simulation."""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
from typing import List, Optional
import os


class SimVisualizer:
    """Real-time and recorded visualization of drone simulation."""
    
    def __init__(self, path_generator, sim_config, drone_config, camera_config):
        self.path_gen = path_generator
        self.sim_config = sim_config
        self.drone_config = drone_config
        self.camera_config = camera_config
        
        # Data storage for plotting
        self.history = {
            'position': [],
            'velocity': [],
            'yaw': [],
            'state': [],
            'steering': [],
            'line_detected': [],
            'cross_track_error': []
        }
        
        # Path points for plotting
        self.path_points = path_generator.get_path_points()
        self.landing_zone = path_generator.landing_zone
        
        # Figure setup
        self.fig = None
        self.axes = None
        self.animation = None
        
    def log_step(self, drone_state: dict, fsm_state: str, steering_cmd: float, 
                 line_detected: bool, cross_track_error: float = 0.0):
        """Log a simulation step for later plotting."""
        self.history['position'].append([drone_state['x'], drone_state['y'], drone_state['z']])
        self.history['velocity'].append([drone_state['vx'], drone_state['vy'], drone_state['vz']])
        self.history['yaw'].append(drone_state['yaw'])
        self.history['state'].append(fsm_state)
        self.history['steering'].append(steering_cmd)
        self.history['line_detected'].append(line_detected)
        self.history['cross_track_error'].append(cross_track_error)
    
    def compute_cross_track_error(self, position: np.ndarray) -> float:
        """Compute minimum distance from drone to path."""
        if len(self.path_points) < 2:
            return 0.0
        
        pos_xy = position[:2]
        diffs = self.path_points[1:] - self.path_points[:-1]
        seg_lens = np.linalg.norm(diffs, axis=1)
        
        # Vector from segment start to drone
        to_drone = pos_xy - self.path_points[:-1]
        
        # Projection onto segment
        proj = np.sum(to_drone * diffs, axis=1) / (seg_lens**2 + 1e-9)
        proj = np.clip(proj, 0, 1)
        
        # Closest point on each segment
        closest = self.path_points[:-1] + proj[:, np.newaxis] * diffs
        dists = np.linalg.norm(pos_xy - closest, axis=1)
        
        return np.min(dists)
    
    def create_live_plot(self):
        """Create the live matplotlib figure with subplots."""
        self.fig = plt.figure(figsize=(16, 10))
        
        # 3D trajectory view
        self.ax_3d = self.fig.add_subplot(2, 3, 1, projection='3d')
        self.ax_3d.set_title('3D Trajectory')
        self.ax_3d.set_xlabel('X (m)')
        self.ax_3d.set_ylabel('Y (m)')
        self.ax_3d.set_zlabel('Z (m)')
        
        # Top-down view
        self.ax_top = self.fig.add_subplot(2, 3, 2)
        self.ax_top.set_title('Top-Down View')
        self.ax_top.set_xlabel('X (m)')
        self.ax_top.set_ylabel('Y (m)')
        self.ax_top.set_aspect('equal')
        self.ax_top.grid(True, alpha=0.3)
        
        # Simulated camera view
        self.ax_cam = self.fig.add_subplot(2, 3, 3)
        self.ax_cam.set_title('Camera View (Processed)')
        self.ax_cam.axis('off')
        
        # Altitude profile
        self.ax_alt = self.fig.add_subplot(2, 3, 4)
        self.ax_alt.set_title('Altitude vs Time')
        self.ax_alt.set_xlabel('Time (s)')
        self.ax_alt.set_ylabel('Z (m)')
        self.ax_alt.grid(True, alpha=0.3)
        
        # Cross-track error
        self.ax_err = self.fig.add_subplot(2, 3, 5)
        self.ax_err.set_title('Cross-Track Error')
        self.ax_err.set_xlabel('Time (s)')
        self.ax_err.set_ylabel('Error (m)')
        self.ax_err.grid(True, alpha=0.3)
        
        # State timeline
        self.ax_state = self.fig.add_subplot(2, 3, 6)
        self.ax_state.set_title('Flight State')
        self.ax_state.set_xlabel('Time (s)')
        self.ax_state.set_ylabel('State')
        self.ax_state.grid(True, alpha=0.3)
        
        # Plot static elements
        self._plot_static_elements()
        
        plt.tight_layout()
        return self.fig
    
    def _plot_static_elements(self):
        """Plot path and landing zone (static)."""
        # 3D path
        self.ax_3d.plot(self.path_points[:, 0], self.path_points[:, 1], 
                       np.full(len(self.path_points), 0.02), 
                       'g--', alpha=0.5, linewidth=2, label='Path')
        self.ax_3d.scatter([self.landing_zone[0]], [self.landing_zone[1]], [0.02],
                          c='red', s=100, marker='*', label='Landing Zone')
        
        # Top-down path
        self.ax_top.plot(self.path_points[:, 0], self.path_points[:, 1], 
                        'g--', alpha=0.5, linewidth=2, label='Path')
        self.ax_top.scatter([self.landing_zone[0]], [self.landing_zone[1]],
                           c='red', s=150, marker='*', label='Landing Zone')
        # Landing zone circle
        circle = Circle(self.landing_zone, self.path_gen.get_landing_zone_radius(),
                       fill=False, edgecolor='red', linestyle='--', alpha=0.5)
        self.ax_top.add_patch(circle)
        
        # Set limits with margin
        margin = 2.0
        x_min, x_max = self.path_points[:, 0].min(), self.path_points[:, 0].max()
        y_min, y_max = self.path_points[:, 1].min(), self.path_points[:, 1].max()
        self.ax_top.set_xlim(x_min - margin, x_max + margin)
        self.ax_top.set_ylim(y_min - margin, y_max + margin)
        
        self.ax_3d.set_xlim(x_min - margin, x_max + margin)
        self.ax_3d.set_ylim(y_min - margin, y_max + margin)
        self.ax_3d.set_zlim(0, self.drone_config.hover_height + 1)
        
        self.ax_top.legend(loc='upper right')
        self.ax_3d.legend(loc='upper right')
    
    def update_frame(self, frame_idx: int, drone_sim, image_processor, 
                     state_machine, steering_ctrl, camera):
        """Update animation frame."""
        if frame_idx >= len(self.history['position']):
            return
        
        # Get current data
        pos = self.history['position'][frame_idx]
        yaw = self.history['yaw'][frame_idx]
        state = self.history['state'][frame_idx]
        steering = self.history['steering'][frame_idx]
        line_det = self.history['line_detected'][frame_idx]
        
        # Clear dynamic plots
        for coll in self.ax_3d.collections[:]:
            coll.remove()
        for coll in self.ax_top.collections[:]:
            coll.remove()
        for patch in self.ax_top.patches[:]:
            patch.remove()
        self.ax_cam.clear()
        self.ax_alt.clear()
        self.ax_err.clear()
        self.ax_state.clear()
        
        # Re-plot static elements
        self._plot_static_elements()
        
        # Drone position (3D)
        self.ax_3d.scatter([pos[0]], [pos[1]], [pos[2]], c='blue', s=80, marker='o')
        # Drone orientation arrow
        arrow_len = 0.5
        self.ax_3d.quiver(pos[0], pos[1], pos[2],
                         arrow_len * np.cos(yaw), arrow_len * np.sin(yaw), 0,
                         color='blue', arrow_length_ratio=0.2)
        
        # Drone trail
        if frame_idx > 1:
            trail = np.array(self.history['position'][:frame_idx+1])
            self.ax_3d.plot(trail[:, 0], trail[:, 1], trail[:, 2], 'b-', alpha=0.6, linewidth=1)
            self.ax_top.plot(trail[:, 0], trail[:, 1], 'b-', alpha=0.6, linewidth=1)
        
        # Top-down drone
        self.ax_top.scatter([pos[0]], [pos[1]], c='blue', s=80, marker='o')
        self.ax_top.arrow(pos[0], pos[1], 
                         0.5 * np.cos(yaw), 0.5 * np.sin(yaw),
                         head_width=0.2, head_length=0.2, fc='blue', ec='blue')
        
        # Simulated camera view
        cam_img = camera.capture(np.array(pos), yaw)
        processed = image_processor.process(cam_img)
        self.ax_cam.imshow(processed.binary_mask, cmap='gray')
        self.ax_cam.set_title(f'Camera View\nLine: {"Yes" if line_det else "No"} | Steering: {steering:.2f}')
        self.ax_cam.axis('off')
        
        # Altitude plot
        times = np.arange(len(self.history['position'])) * self.sim_config.dt
        alts = [p[2] for p in self.history['position']]
        self.ax_alt.plot(times[:frame_idx+1], alts[:frame_idx+1], 'b-')
        self.ax_alt.axhline(y=self.drone_config.hover_height, color='g', linestyle='--', alpha=0.5)
        self.ax_alt.set_xlim(0, max(times[-1], 10))
        self.ax_alt.set_ylim(-0.1, self.drone_config.hover_height + 0.5)
        
        # Cross-track error
        errors = self.history['cross_track_error']
        self.ax_err.plot(times[:frame_idx+1], errors[:frame_idx+1], 'r-')
        self.ax_err.set_xlim(0, max(times[-1], 10))
        if errors:
            self.ax_err.set_ylim(-0.1, max(max(errors)*1.2, 0.5))
        
        # State timeline
        states = self.history['state']
        state_map = {'HOVER': 0, 'CRUISE': 1, 'ROTATION': 2, 'LAND': 3, 'LANDED': 4}
        state_vals = [state_map.get(s, 0) for s in states]
        self.ax_state.plot(times[:frame_idx+1], state_vals[:frame_idx+1], 'k-', linewidth=2)
        self.ax_state.set_yticks([0, 1, 2, 3, 4])
        self.ax_state.set_yticklabels(['HOVER', 'CRUISE', 'ROTATION', 'LAND', 'LANDED'])
        self.ax_state.set_xlim(0, max(times[-1], 10))
        self.ax_state.set_ylim(-0.5, 4.5)
        
        # Overall title
        self.fig.suptitle(f'Parrot Mambo Line Follower | t={times[frame_idx]:.1f}s | State: {state}', 
                         fontsize=14)
        
        return self.fig
    
    def run_animation(self, drone_sim, image_processor, state_machine, steering_ctrl, camera,
                     interval: int = 50, save_path: Optional[str] = None):
        """Run the animation."""
        self.create_live_plot()
        
        def animate(frame):
            return self.update_frame(frame, drone_sim, image_processor, 
                                   state_machine, steering_ctrl, camera)
        
        n_frames = len(self.history['position'])
        self.animation = animation.FuncAnimation(
            self.fig, animate, frames=n_frames, interval=interval, blit=False, repeat=False
        )
        
        if save_path:
            print(f"Saving animation to {save_path}...")
            self.animation.save(save_path, writer='ffmpeg', fps=1000//interval)
            print("Done!")
        else:
            plt.show()
    
    def save_plots(self, output_dir: str):
        """Save final summary plots."""
        os.makedirs(output_dir, exist_ok=True)
        
        times = np.arange(len(self.history['position'])) * self.sim_config.dt
        
        # 1. Trajectory plot
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        pos = np.array(self.history['position'])
        axes[0,0].plot(self.path_points[:, 0], self.path_points[:, 1], 'g--', alpha=0.5, label='Path')
        axes[0,0].plot(pos[:, 0], pos[:, 1], 'b-', label='Drone')
        axes[0,0].scatter([self.landing_zone[0]], [self.landing_zone[1]], c='red', s=100, marker='*', label='Landing')
        axes[0,0].set_aspect('equal')
        axes[0,0].set_title('2D Trajectory')
        axes[0,0].set_xlabel('X (m)')
        axes[0,0].set_ylabel('Y (m)')
        axes[0,0].legend()
        axes[0,0].grid(True, alpha=0.3)
        
        # 2. Altitude
        axes[0,1].plot(times, pos[:, 2], 'b-')
        axes[0,1].axhline(y=self.drone_config.hover_height, color='g', linestyle='--', label='Target')
        axes[0,1].set_title('Altitude')
        axes[0,1].set_xlabel('Time (s)')
        axes[0,1].set_ylabel('Z (m)')
        axes[0,1].legend()
        axes[0,1].grid(True, alpha=0.3)
        
        # 3. Cross-track error
        errors = self.history['cross_track_error']
        axes[1,0].plot(times, errors, 'r-')
        axes[1,0].set_title('Cross-Track Error')
        axes[1,0].set_xlabel('Time (s)')
        axes[1,0].set_ylabel('Error (m)')
        axes[1,0].grid(True, alpha=0.3)
        
        # 4. Velocity
        vel = np.array(self.history['velocity'])
        axes[1,1].plot(times, vel[:, 0], label='Vx')
        axes[1,1].plot(times, vel[:, 1], label='Vy')
        axes[1,1].plot(times, vel[:, 2], label='Vz')
        axes[1,1].set_title('Velocity Components')
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Velocity (m/s)')
        axes[1,1].legend()
        axes[1,1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'trajectory_summary.png'), dpi=150)
        plt.close()
        
        # State timeline
        fig, ax = plt.subplots(figsize=(10, 3))
        states = self.history['state']
        state_map = {'HOVER': 0, 'CRUISE': 1, 'ROTATION': 2, 'LAND': 3, 'LANDED': 4}
        state_vals = [state_map.get(s, 0) for s in states]
        ax.plot(times, state_vals, 'k-', linewidth=2)
        ax.set_yticks([0, 1, 2, 3, 4])
        ax.set_yticklabels(['HOVER', 'CRUISE', 'ROTATION', 'LAND', 'LANDED'])
        ax.set_xlabel('Time (s)')
        ax.set_title('State Timeline')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, 'state_timeline.png'), dpi=150)
        plt.close()
        
        print(f"Plots saved to {output_dir}")
    
    def print_statistics(self):
        """Print final statistics."""
        pos = np.array(self.history['position'])
        times = np.arange(len(pos)) * self.sim_config.dt
        errors = self.history['cross_track_error']
        states = self.history['state']
        
        print("\n" + "="*50)
        print("SIMULATION STATISTICS")
        print("="*50)
        print(f"Total time: {times[-1]:.1f} s")
        print(f"Total steps: {len(pos)}")
        print(f"Final position: ({pos[-1,0]:.2f}, {pos[-1,1]:.2f}, {pos[-1,2]:.2f})")
        print(f"Path length: {np.sum(np.linalg.norm(np.diff(self.path_points, axis=0), axis=1)):.2f} m")
        print(f"Distance traveled: {np.sum(np.linalg.norm(np.diff(pos[:,:2], axis=0), axis=1)):.2f} m")
        print(f"Mean cross-track error: {np.mean(errors):.3f} m")
        print(f"Max cross-track error: {np.max(errors):.3f} m")
        print(f"RMS cross-track error: {np.sqrt(np.mean(np.array(errors)**2)):.3f} m")
        
        # State durations
        print("\nState durations:")
        for state_name in ['HOVER', 'CRUISE', 'ROTATION', 'LAND', 'LANDED']:
            count = sum(1 for s in states if s == state_name)
            duration = count * self.sim_config.dt
            print(f"  {state_name}: {duration:.1f} s ({count} steps)")
        
        # Landing accuracy
        landing_dist = np.linalg.norm(pos[-1, :2] - self.landing_zone)
        print(f"\nLanding accuracy: {landing_dist:.3f} m from target")
        print("="*50)