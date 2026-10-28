# Simulated Camera & Image Processing
# Author: Rhutvik

"""Simulated downward-facing camera with color thresholding and region analysis.

Mirrors the original Simulink image processing pipeline:
1. Y1UY2V → RGB conversion (simulated)
2. HSV color thresholding for line detection
3. 3-region analysis (up-left, up-right, center)
4. Binary signal output for steering logic
"""

import numpy as np
import cv2
from dataclasses import dataclass
from typing import Tuple, Optional
from config import camera, image_proc, path_cfg


@dataclass
class RegionResult:
    """Result of region analysis."""
    left_active: bool
    center_active: bool
    right_active: bool
    left_pixels: int
    center_pixels: int
    right_pixels: int
    binary_mask: np.ndarray


class SimulatedCamera:
    """Simulates the Parrot Mambo downward-facing camera."""
    
    def __init__(self, world_path: 'PathGenerator'):
        self.world_path = world_path
        self.width = camera.width
        self.height = camera.height
        
    def capture(self, drone_position: np.ndarray, drone_yaw: float) -> np.ndarray:
        """
        Generate synthetic camera frame based on drone position over path.
        
        Args:
            drone_position: [x, y, z] in world frame (m)
            drone_yaw: Heading angle (rad)
            
        Returns:
            RGB image (height, width, 3) as uint8
        """
        # Create base image (floor texture)
        img = np.full((self.height, self.width, 3), path_cfg.background_color, dtype=np.uint8)
        
        # Project path onto camera image plane
        self._draw_path_on_image(img, drone_position, drone_yaw)
        
        # Add noise for realism
        noise = np.random.randint(-10, 10, img.shape, dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        return img
    
    def _draw_path_on_image(self, img: np.ndarray, pos: np.ndarray, yaw: float):
        """Draw the path on the camera image based on drone pose."""
        # Camera looks straight down
        # World frame: X forward, Y left, Z up
        # Camera frame: X right (image column), Y down (image row), Z forward (optical axis)
        # Transformation: cam_x = world_y, cam_y = -world_x (for yaw=0)
        
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # Rotation from world to camera frame (yaw around Z)
        # World to body: body_x = cos*world_x + sin*world_y, body_y = -sin*world_x + cos*world_y
        # Camera aligned with body but X right, Y down: cam_x = body_y, cam_y = -body_x
        # Combined: cam_x = -sin*world_x + cos*world_y, cam_y = -cos*world_x - sin*world_y
        
        for segment in self.world_path.segments:
            pts_world = segment.points  # (N, 2) in world XY
            
            # Translate to drone-relative
            pts_rel = pts_world - pos[:2]
            
            # Transform to camera frame
            # cam_x = -sin_yaw * rel_x + cos_yaw * rel_y
            # cam_y = -cos_yaw * rel_x - sin_yaw * rel_y
            cam_x = -sin_yaw * pts_rel[:, 0] + cos_yaw * pts_rel[:, 1]
            cam_y = -cos_yaw * pts_rel[:, 0] - sin_yaw * pts_rel[:, 1]
            
            # Project to image pixels
            altitude = max(pos[2], 0.1)
            scale_x = self.width / (2 * altitude * np.tan(np.deg2rad(camera.fov_horizontal / 2)))
            scale_y = self.height / (2 * altitude * np.tan(np.deg2rad(camera.fov_vertical / 2)))
            
            pts_img = np.zeros_like(pts_rel)
            pts_img[:, 0] = self.width / 2 + cam_x * scale_x   # cam_x -> column (right is positive)
            pts_img[:, 1] = self.height / 2 + cam_y * scale_y  # cam_y -> row (down is positive)
            
            # Draw thick line for path
            pts_int = pts_img.astype(np.int32)
            for i in range(len(pts_int) - 1):
                cv2.line(img, tuple(pts_int[i]), tuple(pts_int[i+1]), path_cfg.path_color, 
                        thickness=max(1, int(path_cfg.path_width * scale_x)))


class ImageProcessor:
    """Processes camera frames to detect line position."""
    
    def __init__(self):
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (image_proc.kernel_size, image_proc.kernel_size)
        )
        
        # Region boundaries (normalized 0-1)
        self.top_y = int(image_proc.region_top_frac * camera.height)
        self.left_x = int(image_proc.region_left_frac * camera.width)
        self.right_x = int(image_proc.region_right_frac * camera.width)
    
    def process(self, frame: np.ndarray) -> RegionResult:
        """
        Process frame through thresholding and region analysis.
        
        Args:
            frame: RGB image (H, W, 3) uint8
            
        Returns:
            RegionResult with boolean flags for each region
        """
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # Threshold for path color (red default, handles HSV wrap)
        mask1 = cv2.inRange(hsv, np.array(image_proc.hsv_lower), np.array(image_proc.hsv_upper))
        mask2 = cv2.inRange(hsv, np.array(image_proc.hsv_lower2), np.array(image_proc.hsv_upper2))
        binary_mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological cleanup
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, self.kernel, 
                                        iterations=image_proc.morph_iterations)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, self.kernel, 
                                        iterations=image_proc.morph_iterations)
        
        # Analyze 3 regions in top portion
        roi = binary_mask[:self.top_y, :]
        
        left_roi = roi[:, :self.left_x]
        center_roi = roi[:, self.left_x:self.right_x]
        right_roi = roi[:, self.right_x:]
        
        left_pixels = cv2.countNonZero(left_roi)
        center_pixels = cv2.countNonZero(center_roi)
        right_pixels = cv2.countNonZero(right_roi)
        
        threshold = image_proc.min_pixels_threshold
        
        return RegionResult(
            left_active=left_pixels > threshold,
            center_active=center_pixels > threshold,
            right_active=right_pixels > threshold,
            left_pixels=left_pixels,
            center_pixels=center_pixels,
            right_pixels=right_pixels,
            binary_mask=binary_mask
        )
    
    def get_steering_signal(self, result: RegionResult) -> int:
        """
        Convert region analysis to steering command (Chart 2 logic).
        
        Returns:
            -1: Yaw left, 0: Straight, 1: Yaw right, -2: No line (land trigger)
        """
        if not (result.left_active or result.center_active or result.right_active):
            return -2  # No line detected -> land
        
        if result.left_active and not result.center_active and not result.right_active:
            return -1  # Line on left -> yaw left
        elif result.right_active and not result.center_active and not result.left_active:
            return 1   # Line on right -> yaw right
        else:
            return 0   # Center or multiple -> straight


# For testing without full simulation
def create_test_frame() -> np.ndarray:
    """Create a test frame with a curved path."""
    img = np.full((camera.height, camera.width, 3), (100, 100, 100), dtype=np.uint8)
    
    # Draw a test S-curve
    pts = []
    for i in range(50):
        x = i * 6
        y = int(120 + 60 * np.sin(i * 0.2))
        pts.append([x, y])
    pts = np.array(pts, dtype=np.int32)
    cv2.polylines(img, [pts], False, (255, 0, 0), 8)
    
    return img