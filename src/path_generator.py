# Test Path Generator
# Author: Rhutvik

"""Generates test paths for line following simulation."""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class PathSegment:
    """A segment of the path (polyline)."""
    points: np.ndarray  # (N, 2) world XY coordinates


@dataclass
class PathGenerator:
    """Generates various test paths."""
    
    segments: List[PathSegment]
    landing_zone: np.ndarray  # Center of landing circle
    
    def __init__(self, path_type: str = "s_curve"):
        self.segments = []
        self.landing_zone = np.array([0.0, 0.0])
        self._generate(path_type)
    
    def _generate(self, path_type: str):
        """Generate path based on type."""
        if path_type == "s_curve":
            self._generate_s_curve()
        elif path_type == "figure8":
            self._generate_figure8()
        elif path_type == "sharp_turns":
            self._generate_sharp_turns()
        elif path_type == "spiral":
            self._generate_spiral()
        elif path_type == "straight":
            self._generate_straight()
        else:
            self._generate_s_curve()
    
    def _generate_s_curve(self):
        """S-curve path with gentle turns - starts straight for initial tracking."""
        pts = []
        # Start with straight section for 3m so drone can acquire line
        for i in range(20):
            x = i * 0.15
            y = 0.0
            pts.append([x, y])
        # Then gentle S-curve
        for i in range(80):
            x = 3.0 + i * 0.15
            y = 1.5 * np.sin((x - 3.0) * 0.5)  # Gentler curve
            pts.append([x, y])
        
        self.segments = [PathSegment(np.array(pts))]
        self.landing_zone = np.array([pts[-1][0], pts[-1][1]])
    
    def _generate_figure8(self):
        """Figure-8 path."""
        pts = []
        for i in range(200):
            t = i * 0.05 * np.pi
            x = 3.0 * np.sin(t)
            y = 1.5 * np.sin(2 * t)
            pts.append([x + 5, y])  # Offset to start away from origin
        
        self.segments = [PathSegment(np.array(pts))]
        self.landing_zone = np.array([pts[-1][0], pts[-1][1]])
    
    def _generate_sharp_turns(self):
        """Path with 90-degree turns."""
        pts = [
            [0, 0], [3, 0], [3, 2], [6, 2], [6, -1], 
            [9, -1], [9, 3], [12, 3], [12, 0], [15, 0]
        ]
        self.segments = [PathSegment(np.array(pts))]
        self.landing_zone = np.array([15.0, 0.0])
    
    def _generate_spiral(self):
        """Inward spiral path."""
        pts = []
        for i in range(150):
            t = i * 0.1
            r = 5.0 - t * 0.03
            x = r * np.cos(t)
            y = r * np.sin(t)
            pts.append([x + 6, y])  # Offset
        
        self.segments = [PathSegment(np.array(pts))]
        self.landing_zone = np.array([pts[-1][0], pts[-1][1]])
    
    def _generate_straight(self):
        """Simple straight line."""
        pts = [[i * 0.2, 0] for i in range(50)]
        self.segments = [PathSegment(np.array(pts))]
        self.landing_zone = np.array([pts[-1][0], pts[-1][1]])
    
    def get_path_points(self, num_points: int = 500) -> np.ndarray:
        """Get evenly spaced points along entire path."""
        all_pts = []
        for seg in self.segments:
            all_pts.append(seg.points)
        if not all_pts:
            return np.array([[0, 0]])
        full = np.vstack(all_pts)
        
        # Resample to even spacing
        if len(full) <= num_points:
            return full
        
        # Calculate cumulative distances
        diffs = np.diff(full, axis=0)
        dists = np.sqrt(np.sum(diffs**2, axis=1))
        cumdists = np.concatenate([[0], np.cumsum(dists)])
        total_len = cumdists[-1]
        
        # Sample evenly
        target_dists = np.linspace(0, total_len, num_points)
        resampled = np.zeros((num_points, 2))
        for i in range(2):
            resampled[:, i] = np.interp(target_dists, cumdists, full[:, i])
        
        return resampled
    
    def get_landing_zone_radius(self) -> float:
        """Radius of landing detection zone."""
        return 0.5  # meters
    
    def is_over_landing_zone(self, position: np.ndarray) -> bool:
        """Check if drone is over landing zone."""
        dist = np.linalg.norm(position[:2] - self.landing_zone)
        return dist < self.get_landing_zone_radius()


def create_path(path_type: str = "s_curve") -> PathGenerator:
    """Factory function."""
    return PathGenerator(path_type)