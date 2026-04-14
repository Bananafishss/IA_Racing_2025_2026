import time
import math


class StanleyController:
    def __init__(self,
                 k=1.5,
                 max_steer_rad=math.radians(30),
                 ema_alpha=0.3,
                 throttle_base=0.35,
                 min_throttle=0.40,
                 max_throttle=0.60,
                 v_min=0.01,
                 freeze_speed=0.005,
                 freeze_steer_to_zero=True,
                 offset_bias_m=0.0):
        self.k = float(k)
        self.max_steer = float(max_steer_rad)
        self.ema_alpha = float(ema_alpha)

        self.throttle_base = float(throttle_base)
        self.min_throttle = float(min_throttle)
        self.max_throttle = float(max_throttle)

        self.v_min = float(v_min)
        self.freeze_speed = float(freeze_speed)
        self.freeze_steer_to_zero = bool(freeze_steer_to_zero)
        self.offset_bias_m = float(offset_bias_m)

        self.cte_filtered = 0.0
        self.last_steer = 0.0
        self.last_time = None

    def run(self, offset_m, path_angle_deg, current_speed=0.0, current_yaw=0.0):
        if offset_m is None or path_angle_deg is None:
            return 0.0, 0.0

        offset_m = float(offset_m) - self.offset_bias_m

        # EMA on lateral error
        self.cte_filtered = self.ema_alpha * offset_m + (1.0 - self.ema_alpha) * self.cte_filtered

        # Heading error
        theta_path = math.radians(float(path_angle_deg))
        theta_e = (float(current_yaw) - theta_path + math.pi) % (2.0 * math.pi) - math.pi

        # Stanley cross term with speed floor
        v = float(current_speed)
        v_ctrl = max(self.v_min, v)
        cross_term = math.atan2(self.k * self.cte_filtered, v_ctrl)

        delta_cmd = theta_e + cross_term

        # Saturation
        delta_cmd = max(-self.max_steer, min(self.max_steer, delta_cmd))

        # Low-speed freeze
        if v < self.freeze_speed and self.freeze_steer_to_zero:
            delta_cmd = 0.0

        # Rate limit
        now = time.time()
        if self.last_time is not None:
            dt = max(1e-6, now - self.last_time)
            max_change = math.radians(90.0) * dt
            delta_cmd = max(self.last_steer - max_change,
                            min(self.last_steer + max_change, delta_cmd))

        self.last_steer = delta_cmd
        self.last_time = now

        # Normalize steer
        steer_norm = delta_cmd / self.max_steer

        # Throttle (anti-stall)
        if v < self.freeze_speed:
            throttle = max(self.throttle_base, self.min_throttle)
        else:
            throttle = self.throttle_base

        throttle = min(throttle, self.max_throttle)

        # Debug
        print(f"[Stanley] offset_m={offset_m:.4f}, cte_f={self.cte_filtered:.4f}, "
              f"v={v:.3f}, v_ctrl={v_ctrl:.3f}, theta_e={theta_e:.3f}, cross={cross_term:.3f}")
        print(f"[Stanley OUT] steer={steer_norm:.3f}, throttle={throttle:.3f}")

        return steer_norm, throttle

    def shutdown(self):
        pass
