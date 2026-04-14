"""
Camera distortion correction module for Donkey Car
Loads calibration parameters and corrects images in real-time
"""

import pickle
from pathlib import Path
import cv2
import numpy as np


class CameraUndistorter:
    def __init__(self, calibration_file=None):
        """
        Initialize undistorter with calibration parameters
        
        Args:
            calibration_file: Path to calibration pickle file
                            If None, attempts to find it automatically
        """
        if calibration_file is None:
            calibration_file = Path.home() / "camera_calibration.pkl"
        
        calibration_file = Path(calibration_file)
        
        if not calibration_file.exists():
            print(f"WARNING: Calibration file not found: {calibration_file}")
            self.camera_matrix = None
            self.dist_coeffs = None
            self.enabled = False
            return
        
        try:
            with open(calibration_file, 'rb') as f:
                data = pickle.load(f)
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
                self.image_shape = data['image_shape']
                self.enabled = True
                print(f"✓ Calibration parameters loaded from: {calibration_file}")
        except Exception as e:
            print(f"ERROR loading calibration: {e}")
            self.enabled = False
    
    def undistort(self, frame):
        """
        Corrects image distortion
        
        Args:
            frame: Numpy image (BGR or RGB)
        
        Returns:
            Corrected image (same format as input)
        """
        if not self.enabled or self.camera_matrix is None:
            return frame
        
        try:
            undistorted = cv2.undistort(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                None,
                self.camera_matrix
            )
            return undistorted
        except Exception as e:
            print(f"Error during undistort: {e}")
            return frame


# Global instance
undistorter = CameraUndistorter()
