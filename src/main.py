#!/usr/bin/env python3
# Parrot Mambo Line Follower - Main Entry Point
# Author: Rhutvik

"""Main simulation entry point for Parrot Mambo line following replica.

Run with:
    python src/main.py                    # Interactive visualization
    python src/main.py --record           # Record to video
    python src/main.py --headless         # No visualization, just logs
    python src/main.py --path figure8     # Different test path
"""

import argparse
import os
import sys
import numpy as np

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from config import sim, drone
from drone_sim import DroneSimulator, create_initial_state
from image_processing import SimulatedCamera, ImageProcessor
from state_machine import StateMachine, FlightState
from steering import SteeringController
from path_generator import create_path
from visualize import SimVisualizer


def run_simulation(path_type: str = "s_curve", record: bool = False, headless: bool = False):
    """Run the complete simulation."""
    
    # Initialize components
    path_gen = create_path(path_type)
    drone_sim = DroneSimulator(create_initial_state())
    camera = SimulatedCamera(path_gen)
    image_proc = ImageProcessor()
    state_machine = StateMachine()
    steering_ctrl = SteeringController()
    visualizer = SimVisualizer(path_gen, sim, drone, camera)
    
    # Ensure output directories exist
    os.makedirs(sim.output_dir, exist_ok=True)
    os.makedirs(sim.log_dir, exist_ok=True)
    
    print(f"Starting simulation: {path_type} path")
    print(f"Max steps: {sim.max_steps}, dt: {sim.dt}s")
    print(f"Drone: {drone.mass}kg, max_vel: {drone.max_velocity}m/s")
    
    # Initialize - arm the drone for takeoff
    drone_sim.takeoff()
    
    # Simulation loop
    for step in range(sim.max_steps):
        # Get drone state
        state_dict = drone_sim._get_state_dict()
        pos = np.array([state_dict['x'], state_dict['y'], state_dict['z']])
        yaw = state_dict['yaw']
        
        # Capture camera frame
        frame = camera.capture(pos, yaw)
        
        # Process image
        region_result = image_proc.process(frame)
        
        # Steering logic (Chart 2) - continuous yaw rate
        steering_out = steering_ctrl.compute(region_result, sim.dt)
        
        # State machine (Chart 1)
        fsm_state, outputs = state_machine.update(
            sim.dt, 
            steering_out.yaw_rate_cmd,  # Continuous yaw rate
            steering_out.line_detected,
            yaw,
            pos[2]
        )
        
        # Apply control to drone
        drone_sim.step(
            vx_body=outputs.vx_body,
            vy_body=outputs.vy_body,
            vz_world=outputs.vz_world,
            yaw_rate_cmd=outputs.yaw_rate_cmd
        )
        
        # Cross-track error for logging
        cte = visualizer.compute_cross_track_error(pos)
        
        # Log
        drone_sim.log_state(fsm_state.value, steering_out.yaw_command, steering_out.line_detected)
        visualizer.log_step(state_dict, fsm_state.value, steering_out.yaw_command, 
                           steering_out.line_detected, cte)
        
        # Progress
        if step % 300 == 0:
            print(f"  Step {step}/{sim.max_steps} | State: {fsm_state.value} | "
                  f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
                  f"Yaw: {np.degrees(yaw):.0f}° | Steering: {steering_out.yaw_command:.2f}")
        
        # Check for completion
        if fsm_state == FlightState.LANDED:
            print(f"\nLanded at step {step} (t={step*sim.dt:.1f}s)")
            break
        
        # Check if reached end of path
        if path_gen.is_over_landing_zone(pos) and fsm_state != FlightState.HOVER:
            # Let state machine handle landing
            pass
    
    # Save logs
    log_path = os.path.join(sim.log_dir, f'flight_log_{path_type}.csv')
    drone_sim.save_log(log_path)
    print(f"Flight log saved to {log_path}")
    
    # Statistics and plots
    visualizer.print_statistics()
    visualizer.save_plots(sim.output_dir)
    
    # Animation
    if not headless:
        video_path = os.path.join(sim.output_dir, f'simulation_{path_type}.mp4') if record else None
        visualizer.run_animation(drone_sim, image_proc, state_machine, steering_ctrl, camera,
                                interval=int(sim.dt * 1000), save_path=video_path)
    
    return visualizer


def main():
    parser = argparse.ArgumentParser(description='Parrot Mambo Line Follower Simulation')
    parser.add_argument('--path', type=str, default='s_curve',
                       choices=['s_curve', 'figure8', 'sharp_turns', 'spiral', 'straight'],
                       help='Test path type')
    parser.add_argument('--record', action='store_true',
                       help='Record animation to video (requires ffmpeg)')
    parser.add_argument('--headless', action='store_true',
                       help='Run without visualization')
    parser.add_argument('--steps', type=int, default=None,
                       help='Override max simulation steps')
    args = parser.parse_args()
    
    if args.steps:
        sim.max_steps = args.steps
    
    run_simulation(args.path, args.record, args.headless)


if __name__ == '__main__':
    main()