import numpy as np

def smallest_signed_angle(lhs, rhs):
    # vectorized for NumPy arrays
    return (lhs - rhs + np.pi) % (2 * np.pi) - np.pi

class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Integrator_max=10, use_ssa=False):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.integrator_max = Integrator_max
        self.reference = 0.0
        self.use_ssa = use_ssa

        self.accumulated_error = 0
        self.last_error = 0

        self.time_step = 1

    def update(self, current_value: float) -> float:
        error = self.reference - current_value

        if self.use_ssa:
            error = smallest_signed_angle(error)
        
        self.accumulated_error = max(-self.integrator_max, min(self.integrator_max, self.accumulated_error + error * self.time_step))

        P_error = error
        I_error = self.accumulated_error
        D_error = (error - self.last_error) / self.time_step

        self.last_error = error

        return self.Kp * P_error + self.Ki * I_error + self.Kd * D_error

    def setReference(self, reference: float):
        self.reference = reference
        self.accumulated_error = 0
        self.last_error = 0

    def setParameters(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D