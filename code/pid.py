class PDController:
    def __init__(self, Kp, Kd, dt, setpoint=0.0):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0.0
        assert dt > 0, "Time step must be positive"
        self.dt = dt
        self.setpoint = setpoint

    def update(self, measured_value):
        """
        Update the PD controller with the measured value.
        * measured_value: The current value to be controlled.
        Returns the control output.
        """
        error = self.setpoint - measured_value
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp * error + self.Kd * derivative, error, derivative