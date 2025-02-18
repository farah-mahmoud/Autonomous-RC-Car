import numpy as np
import queue
from collections import deque
from threading import Thread
import pigpio

# Constants
LOOK_AHEAD_BUFF = 4
SUB_HARMONIC_COUNT_THRESHOLD = 5

class DebouncePlusGHV4:
    def __init__(self, x0, dx, g, h):
        self.x = x0
        self.dx = dx
        self.g = g
        self.h = h
        self.look_ahead_buff = np.zeros(LOOK_AHEAD_BUFF)
        self.look_ahead_idx = 0
        self.sub_harmonic_count = 0

    def update(self, dt):
        if self.look_ahead_idx == 0:
            self.look_ahead_buff[self.look_ahead_idx] = dt
            self.look_ahead_idx += 1
        else:
            if dt < 1000.0:
                self.look_ahead_buff[self.look_ahead_idx - 1] += dt
            else:
                self.look_ahead_buff[self.look_ahead_idx] = dt + self.look_ahead_buff[self.look_ahead_idx - 1]
                self.look_ahead_idx += 1

            if self.look_ahead_idx >= LOOK_AHEAD_BUFF:
                x_est = self.x + self.dx * self.look_ahead_buff
                residuals = self.look_ahead_buff - x_est
                min_idx = np.argmin(np.abs(residuals))

                if min_idx > 0:
                    x_est_sub_har = self.x / (min_idx + 1.0) + self.dx * self.look_ahead_buff
                    residuals_sub_har = self.look_ahead_buff - x_est_sub_har
                    abs_residuals_sub_har = np.abs(residuals_sub_har)

                    if abs_residuals_sub_har[0] <= abs(residuals[min_idx]):
                        self.sub_harmonic_count += 1
                    else:
                        self.sub_harmonic_count = 0

                    if self.sub_harmonic_count > SUB_HARMONIC_COUNT_THRESHOLD:
                        self.x /= (min_idx + 1.0)
                        residuals[min_idx] = residuals_sub_har[0]
                        min_idx = 0
                        self.sub_harmonic_count = 0

                clip_residual_dt = np.clip(residuals[min_idx] / self.look_ahead_buff[min_idx], -0.6, 0.6)
                self.dx += self.h * clip_residual_dt
                self.x = x_est[min_idx] + self.g * residuals[min_idx]
                result = self.x

                self.look_ahead_buff -= self.look_ahead_buff[min_idx]
                self.look_ahead_buff[:-min_idx-1] = self.look_ahead_buff[min_idx+1:]
                self.look_ahead_idx = LOOK_AHEAD_BUFF - min_idx - 1
            else:
                result = None
        return result

class SpeedSensor:
    SPEED_SENSOR_GPIO = 10
    SPEED_SENSOR_TIMEOUT_MS = 1000

    def __init__(self, gpio=SPEED_SENSOR_GPIO, timeout=SPEED_SENSOR_TIMEOUT_MS):
        self.gpio = gpio
        self.timeout = timeout
        self.raw_delta_t_queue = queue.Queue()
        self.rpm_queue = deque(maxlen=1)
        self.rpm_queue.append(0.0)
        self.last_tick = None
        self.run_thread = False

    def start(self):
        self.run_thread = True
        self.debounce_thread = Thread(target=self.debounce_gh_filter_thread, args=())
        self.debounce_thread.start()
        self.pi = pigpio.pi()
        self.pi.set_mode(self.gpio, pigpio.INPUT)
        self._callback = self.pi.callback(self.gpio, pigpio.RISING_EDGE, self._cbf)
        self.pi.set_watchdog(self.gpio, self.timeout)

    def shutdown(self):
        self._callback.cancel()
        self.pi.set_watchdog(self.gpio, 0)
        self.pi.stop()
        self.run_thread = False

    def get_speed_act(self):
        return self.rpm_queue[-1]

    def _cbf(self, gpio, level, tick):
        if level == 1:  # Rising edge.
            if self.last_tick is not None:
                self.raw_delta_t_queue.put_nowait(pigpio.tickDiff(self.last_tick, tick))
            self.last_tick = tick
        elif level == 2:  # Watchdog timeout.
            self.raw_delta_t_queue.put_nowait(600000)
            self.last_tick = None

    def debounce_gh_filter_thread(self):
        debounce = DebouncePlusGHV4(self.timeout * 1000, 0.0, 0.7, 0.4)
        while self.run_thread:
            try:
                raw_delta_t = self.raw_delta_t_queue.get(block=True, timeout=0.5)
                delta_t = debounce.update(raw_delta_t)
                if delta_t:
                    self.rpm_queue.append(60000.0 / delta_t)
            except Exception:
                pass
