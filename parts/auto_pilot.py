class AutoPilotPart:
    def __init__(self, throttle=0.5):
        self.throttle = throttle

    def run(self, steer, throttle=None):
        if throttle is None:
            throttle = self.throttle
        return steer, throttle

