# DonkeyCar Classical Autopilot with Stanley Controller

## Overview
This repository contains only the custom code developed for an autonomous DonkeyCar based on classical computer vision and control.
It is designed to be used together with a standard DonkeyCar `mycar` application created from the official templates.

The system provides:
- Real-time lane detection.
- A Stanley controller for lateral control.
- Odometry-based speed and yaw estimation.
- A simple autopilot integration with DonkeyCar's drive loop.
- Data logging to CSV and offline plotting utilities.

## Repository Structure
Top-level Python modules:
- `camera_undistort.py` — CameraUndistorter class to apply calibration to incoming frames.
- `lane_detector_realtime.py` — Real-time lane detection (offset and heading estimation).
- `generate_plots.py` — Script to generate perception + control plots from logged CSV data.
- `ina219.py` — INA219 current/voltage sensor helper (optional, if used in your setup).
- `stanley_controller.py` — Standalone implementation of the Stanley controller.
- `test_add_detection.py` — Test script to validate lane detection.
- `test_motor.py` — Test script for the motor/ESC.
- `test_oled.py` — Test script for the OLED display (if present).
- `test_pin.py` — Simple GPIO test.
- `test_servo.py` — Test script for the steering servo.

Custom DonkeyCar parts (in `parts/`):
- `lane_detector_part.py` — Wraps the lane detector as a DonkeyCar part.
- `stanley_controller_part.py` — Wraps the Stanley controller as a DonkeyCar part.
- `odometry_part.py` — Odometry part that estimates current speed and yaw.
- `auto_pilot.py` — Part that maps controller outputs to `pilot/angle` and `pilot/throttle` for the DonkeyCar vehicle.
- `data_logger_part.py` — Logs key signals to CSV for offline analysis.
- `memory_logger_part.py` — Optional part to log internal memory/debug information.

> Note: `manage.py` and `myconfig.py` are **not** included here; they come from the official DonkeyCar templates and live in your standard `mycar` folder.

## Integration into DonkeyCar
These parts are intended to be integrated into the standard DonkeyCar `manage.py` drive loop.
After creating your car with:
```bash
donkey createcar --path ~/mycar
```
you can copy this repository's files into `~/mycar` (or adjust import paths accordingly).

### Imports in `manage.py`
Inside `manage.py`, add the following imports near the top:
```python
from parts.lane_detector_part import LaneDetectorPart
from parts.stanley_controller_part import StanleyControllerPart
from parts.auto_pilot import AutoPilotPart
from parts.odometry_part import OdometryPart
from parts.memory_logger_part import MemoryLoggerPart
from parts.data_logger_part import DataLoggerPart
```

### Parts added to the vehicle
Inside the `drive(cfg, model_path=None, use_joystick=False, model_type=None,
          camera_type='single', meta=[]):` function, after the camera part is added, add the custom parts in this order (conceptually):

1. **Lane detector** — reads camera image and (optionally) steer, outputs offset and angle.
2. **Odometry** — outputs current speed and yaw.
3. **Stanley controller** — consumes offset, angle, speed, and yaw; outputs `steer` and `throttle`.
4. **Data logger** — logs perception and control signals to CSV under `~/mycar/data_plot`.
5. **AutoPilot part** — maps `steer` and `throttle` to `pilot/angle` and `pilot/throttle`.
6. **DriveMode** — chooses between user and pilot commands.

A simplified version of the additions looks like this (names/tuning can be adjusted):
```python
lane_detector = LaneDetectorPart()
V.add(lane_detector,
      inputs=['cam/image_array', 'steer'],
      outputs=['offset_m', 'angle_deg'])

odometry = OdometryPart(
    gpio_pin=13,
    ppr=20,
    wheel_radius_m=0.023,
    ema_alpha=0.3,
    timeout_s=0.5,
)
V.add(odometry,
      inputs=[],
      outputs=['current_speed', 'current_yaw'])

stanley_controller = StanleyControllerPart(
    k=0.8,
    throttle_base=0.33,
    min_throttle=0.33,
    max_throttle=0.35,
)
V.add(stanley_controller,
      inputs=['offset_m', 'angle_deg', 'current_speed', 'current_yaw'],
      outputs=['steer', 'throttle'])

data_logger = DataLoggerPart(base_dir="/home/pi/mycar/data_plot")
V.add(data_logger,
      inputs=['offset_m', 'angle_deg', 'steer', 'throttle',
              'current_speed', 'current_yaw'],
      outputs=[])

auto_pilot = AutoPilotPart()
V.add(auto_pilot,
      inputs=['steer', 'throttle'],
      outputs=['pilot/angle', 'pilot/throttle'])

V.add(DriveMode(cfg.AI_THROTTLE_MULT),
      inputs=['user/mode',
              'user/angle', 'user/throttle',
              'pilot/angle', 'pilot/throttle'],
      outputs=['steering', 'throttle'])

V.mem.put('user/mode', 'local_pilot')
```
This builds a purely classical autopilot pipeline on top of the DonkeyCar vehicle loop, without using the neural network training or Tub-based datasets.

## Data Logging and Plotting
The `DataLoggerPart` writes CSV files with the key signals needed for offline analysis, for example:
- Timestamp
- Lateral offset (meters)
- Heading angle (degrees)
- Steering command
- Throttle command
- Current speed
- Current yaw

These CSV files are saved under a directory such as:
- `~/mycar/data_plot/data_*.csv`

The script `generate_plots.py` loads the latest CSV and produces a figure with two subplots:
- **Perception**: offset and angle over time.
- **Control**: steer and throttle over time.

Example usage (from `~/mycar`):
```bash
python generate_plots.py
```
The script creates a folder like `plots_data_1` and saves a PNG file with the combined plot.

## Camera Calibration and Undistortion
`camera_undistort.py` provides a `CameraUndistorter` class that:
- Loads calibration parameters from `~/camera_calibration.pkl` (camera matrix, distortion coefficients, image shape).
- Applies `cv2.undistort` to each frame if calibration data is available.
- Falls back to returning the original frame if the calibration file is missing or invalid.

You can integrate this undistortion step inside your camera pipeline or directly in your lane detector, depending on performance and image quality requirements.

## Stanley Controller
The Stanley controller is a geometric path tracking controller originally popularized by the Stanford "Stanley" vehicle.
In this project, it uses:
- Lateral offset from the lane center.
- Heading error relative to the lane.
- Current speed estimate.

The output is a steering command that keeps the car close to the lane center and stable at low speeds suitable for DonkeyCar-scale robots.

Key tuning parameters:
- `k`: gain on the cross-track error term.
- Speed softening term to avoid excessive steering at very low speeds.
- Saturation/limits on the steering command and throttle range.

## How to Run
On the Raspberry Pi (inside your DonkeyCar virtual environment):
```bash
cd ~/mycar
python manage.py drive --js
```
This starts the DonkeyCar vehicle loop with your custom parts integrated.
You can then connect to the web interface (or joystick) to switch between user and autopilot modes and observe the behavior.

## Notes
- This project deliberately avoids the built-in deep-learning pipeline of DonkeyCar and focuses on interpretability and classical control.
- All autonomy logic is isolated in clearly defined parts under the `parts/` directory, making it easier to test and extend.
- You are free to adapt thresholds, gains, and logging formats to match your track and hardware.

## Related Projects and References

This project is part of a series of DonkeyCar projects developed at IMT Atlantique.  
Previous years' work and the base platform are available here:

- **DonkeyCar base project**  
  https://github.com/autorope/donkeycar  
  Open‑source hardware and software platform for building small‑scale self‑driving cars.

- **DonkeyCar official documentation / installation tutorial**  
  https://docs.donkeycar.com/ 

- **2022–2023 IMT Atlantique project**  
  https://github.com/Rom-1T/ia_racing_imt

- **2023–2024 IMT Atlantique project**  
  https://github.com/RoryMaillard/ia_racing_imt_2023

- **2024–2025 IMT Atlantique project**  
  https://gitlab.imt-atlantique.fr/procom-ia-racing/2024-procom-ia-racing
  ## Structure of our Team
  Perception Groupe: WANG Lixi, ZHANG Chen
  
  Controller Groupe: Anthony CENZANO
  
  Calibration Groupe: Zaineb MAHMOUDI, Wissal DAHANI
