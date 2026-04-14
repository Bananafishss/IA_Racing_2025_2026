from stanley_controller import StanleyController

class StanleyControllerPart:
    def __init__(self, k=1.0, throttle_base=0.35, min_throttle=0.40, max_throttle=0.60):
        self.controller = StanleyController(
            k=k,
            throttle_base=throttle_base,
            min_throttle=min_throttle,
            max_throttle=max_throttle
        )
        self.frame_count = 0

    def run(self, offset_m, angle_deg, current_speed, current_yaw=0.0):
        self.frame_count += 1

        if offset_m is None or angle_deg is None:
            return 0.0, 0.0

        if current_speed is None:
            current_speed = 0.0
        if current_yaw is None:
            current_yaw = 0.0

        steer, throttle = self.controller.run(offset_m, angle_deg, current_speed, current_yaw)
        return steer, throttle

    def shutdown(self):
        self.controller.shutdown()
