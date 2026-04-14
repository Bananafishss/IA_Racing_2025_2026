from lane_detector_realtime import LaneDetector

class LaneDetectorPart:
    def __init__(self, hold_frames=5):
        self.detector = LaneDetector()
        self.hold_frames = hold_frames
        self._last_offset_m = None
        self._last_angle_deg = None
        self._miss_count = 0
        self._frame_count = 0

    def run(self, image, last_steer):
        self._frame_count += 1

        if image is None:
            return None, None

        if last_steer is None:
            last_steer = 0.0

        result = self.detector.process(frame_bgr=image, last_steer=last_steer)

        if result is None or not isinstance(result, dict):
            self._miss_count += 1
            if (self._miss_count <= self.hold_frames and 
                self._last_offset_m is not None and 
                self._last_angle_deg is not None):
                return self._last_offset_m, self._last_angle_deg
            return None, None

        offset_m = result.get("offset_m")
        angle_deg = result.get("lane_center_angle_deg")

        if offset_m is not None and angle_deg is not None:
            self._last_offset_m = float(offset_m)
            self._last_angle_deg = float(angle_deg)
            self._miss_count = 0
            return self._last_offset_m, self._last_angle_deg

        self._miss_count += 1

        if (self._miss_count <= self.hold_frames and 
            self._last_offset_m is not None and 
            self._last_angle_deg is not None):
            return self._last_offset_m, self._last_angle_deg

        return None, None

    def shutdown(self):
        pass
