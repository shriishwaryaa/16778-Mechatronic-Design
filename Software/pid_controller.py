class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.integral = self.last_error = 0

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.last_error = error
        return output