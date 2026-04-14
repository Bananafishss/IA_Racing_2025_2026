# data_logger_part.py
import os
import csv
import time

class DataLoggerPart:
    def __init__(self, base_dir="/home/pi/mycar/data_plot"):
        self.base_dir = base_dir
        os.makedirs(base_dir, exist_ok=True)
        
        self.run_number = self._find_next_run_number()
        self.csv_filename = f"data_{self.run_number}.csv"
        self.csv_path = os.path.join(base_dir, self.csv_filename)
        
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'offset_m', 'angle_deg', 'steer', 
                'throttle', 'current_speed', 'current_yaw'
            ])
        
        self.start_time = time.time()
        self.frame_count = 0
        print(f"[DataLogger] Logging to: {self.csv_filename}")

    def _find_next_run_number(self):
        existing = [f for f in os.listdir(self.base_dir) if f.startswith('data_') and f.endswith('.csv')]
        if not existing:
            return 1
        
        numbers = [int(f.replace('data_', '').replace('.csv', '')) for f in existing if f[5:-4].isdigit()]
        return max(numbers) + 1 if numbers else 1

    def run(self, offset_m, angle_deg, steer, throttle, current_speed, current_yaw):
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        
        row = [
            f"{elapsed:.3f}",
            f"{offset_m:.6f}" if offset_m is not None else "",
            f"{angle_deg:.3f}" if angle_deg is not None else "",
            f"{steer:.6f}" if steer is not None else "",
            f"{throttle:.6f}" if throttle is not None else "",
            f"{current_speed:.6f}" if current_speed is not None else "",
            f"{current_yaw:.6f}" if current_yaw is not None else ""
        ]
        
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        if self.frame_count % 100 == 0:
            print(f"[DataLogger] Frame {self.frame_count}: {elapsed:.1f}s logged")

    def shutdown(self):
        print(f"[DataLogger] Saved {self.frame_count} frames to {self.csv_filename}")
