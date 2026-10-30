"""Short-window standstill evidence for operator arming and photography."""
from collections import deque
import math
from statistics import median


class StationaryDetector:
    def __init__(self):
        self.samples = deque()
        self.last_stamp = None

    def update(self, now, stamp, age, speed, yaw_rate, x, y, yaw):
        values = (now, stamp, age, speed, yaw_rate, x, y, yaw)
        if not all(math.isfinite(v) for v in values) or not -0.1 <= age < 3.:
            self.samples.clear()
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.samples.clear()
            return
        self.last_stamp = stamp
        if self.samples and now-self.samples[-1][0] > 1.:
            self.samples.clear()
        self.samples.append((now, abs(speed), abs(yaw_rate), x, y, yaw))
        while self.samples and now-self.samples[0][0] > 1.2:
            self.samples.popleft()

    def ready(self, now):
        rows = list(self.samples)
        if len(rows) < 3 or now-rows[-1][0] > 1. or rows[-1][0]-rows[0][0] < .6:
            return False
        speeds = sorted(r[1] for r in rows)
        rates = sorted(r[2] for r in rows)
        # A few measured LIO velocity spikes must not restart a continuous
        # timer. Sustained movement, large spikes and pose drift still reject.
        if median(speeds) >= .035 or speeds[math.ceil(.8*len(rows))-1] >= .08 or speeds[-1] >= .15:
            return False
        if median(rates) >= .05 or rates[-1] >= .15:
            return False
        for i, a in enumerate(rows):
            for b in rows[i+1:]:
                if math.hypot(a[3]-b[3], a[4]-b[4]) > .04:
                    return False
                if abs(math.atan2(math.sin(a[5]-b[5]), math.cos(a[5]-b[5]))) > .06:
                    return False
        return True
