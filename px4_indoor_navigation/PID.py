import time

class PID:
    """PID Controller"""

    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.I_TERM_IS_ACTIVE = False
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback"""
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            # Windup Guard
            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            # If I_TERM_IS_ACTIVE is False, reset the integral term
            if not self.I_TERM_IS_ACTIVE:
                self.ITerm = 0.0

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Sets the proportional gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Sets the integral gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Sets the derivative gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Sets the windup guard"""
        self.windup_guard = windup

    def resetITerm(self):
        """Resets the integral term"""
        self.ITerm = 0.0

    def setSampleTime(self, sample_time):
        """Sets the sample time"""
        self.sample_time = sample_time
