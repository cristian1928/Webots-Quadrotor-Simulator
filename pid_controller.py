def clamp(value, low, high):
    """Clamps a value to a given range."""
    return max(low, min(value, high))

class PID:
    """A simple Proportional-Integral-Derivative (PID) controller."""
    def __init__(self, kp, ki, kd, integral_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self._previous_measurement = 0.0
        self._is_first_run = True

    def update(self, setpoint, current_measurement, time_delta_seconds):
        """
        Calculates the PID control output.

        Args:
            setpoint: The desired value.
            current_measurement: The current value from sensors.
            time_delta_seconds: The time elapsed since the last update.

        Returns:
            The control output value.
        """
        error = setpoint - current_measurement
        
        self.integral += (error * time_delta_seconds)
        if self.integral_limit is not None:
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        if self._is_first_run or time_delta_seconds == 0:
            derivative_of_measurement = 0.0
            self._is_first_run = False
        else:
            derivative_of_measurement = (current_measurement - self._previous_measurement) / time_delta_seconds
        
        self._previous_measurement = current_measurement
        
        proportional_term = self.kp * error
        integral_term = self.ki * self.integral
        derivative_term = -self.kd * derivative_of_measurement

        return proportional_term + integral_term + derivative_term

    def reset(self):
        """Resets the integral and derivative states of the controller."""
        self.integral = 0.0
        self._previous_measurement = 0.0
        self._is_first_run = True
