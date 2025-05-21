class PIDController:
    def __init__(self, Kp, Kd, Ki, dt, setpoint=0.0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integral = 0.0
        self.prev_error = 0.0
        assert dt > 0, "Time step must be positive"
        self.dt = dt
        self.setpoint = setpoint

    def update(self, measured_value):
        """
        Update the PID controller with the measured value.
        * measured_value: The current value to be controlled.
        Returns the control output.
        """
        error = self.setpoint - measured_value
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        self.integral += error * self.dt
        return self.Kp * error + self.Kd * derivative + self.Ki * self.integral, error, derivative