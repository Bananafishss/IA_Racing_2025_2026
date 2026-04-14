import os
import cv2
import numpy as np
from datetime import datetime

class MemoryLoggerPart:
    def __init__(self, base_dir="/home/pi/mycar/memory"):
        self.base_dir = base_dir
        self.dir_raw = os.path.join(base_dir, "images_raw")
        self.dir_vec = os.path.join(base_dir, "images_vec")
        os.makedirs(self.dir_raw, exist_ok=True)
        os.makedirs(self.dir_vec, exist_ok=True)
        self.frame_count = 0

    def run(self, image, steer, throttle):
        if image is None:
            return

        self.frame_count += 1
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        fname = f"{self.frame_count:06d}_{ts}.jpg"

        # Save raw image
        raw_path = os.path.join(self.dir_raw, fname)
        cv2.imwrite(raw_path, image)

        # Draw vector overlay
        vec_img = image.copy()
        h, w = vec_img.shape[:2]
        x0, y0 = w // 2, int(h * 0.9)
        max_len = 80
        length = max_len * float(throttle)

        angle_deg = 90 - float(steer) * 90
        angle = np.deg2rad(angle_deg)
        dx = length * np.cos(angle)
        dy = -length * np.sin(angle)
        x1, y1 = int(x0 + dx), int(y0 + dy)

        cv2.arrowedLine(vec_img, (x0, y0), (x1, y1), (0, 0, 255), 2, tipLength=0.3)
        cv2.circle(vec_img, (x0, y0), 3, (255, 0, 0), -1)
        text = f"s={steer:.2f}, t={throttle:.2f}"
        cv2.putText(vec_img, text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        vec_path = os.path.join(self.dir_vec, fname)
        cv2.imwrite(vec_path, vec_img)

    def shutdown(self):
        pass
