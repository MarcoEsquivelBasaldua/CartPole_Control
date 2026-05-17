import numpy as np

class PIDController:
    def __init__(self):
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.integral = 0.0
        self.prev_error = 0.0

    def compute_control(self, setpoint: float, current_value: np.ndarray, dt: float) -> float:
        setpoint = np.array([[setpoint],  # x cart position setpoint
                             [0.0]])      # pole angle setpoint (upright)
        
        current_value = np.array([[current_value[0, 0]],  # x cart position
                                  [current_value[1, 0]]])  # pole angle

        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return control_signal
    


def wrap_to_pi(angle):
    """
     Wraps an angle in radians to the range (-pi, pi].
    """
    # Wrap to [0, 2*pi)
    angle = angle % (2 * np.pi)
    # Shift to (-pi, pi]
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle