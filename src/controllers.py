import numpy as np

class PIDController:
    def __init__(self):
        self.KpX = 1.0
        self.KiX = 0.0
        self.KdX = 0.0
        self.KpTheta = 10.0
        self.KiTheta = 0.0
        self.KdTheta = 0.0
        self.integralX = 0.0
        self.integralTheta = 0.0
        self.prevErrorX = 0.0
        self.prevErrorTheta = 0.0

    def compute_control(self, setpoint: float, currentValue: np.ndarray, dt: float) -> float:
        setpointVect = np.array([[setpoint],  # x cart position setpoint
                                 [0.0]])      # pole angle setpoint (upright)
        
        currentValueVect = np.array([[currentValue[0, 0]],  # x cart position
                                    [currentValue[1, 0]]])  # pole angle

        # Outer loop for cart position control
        errorX = setpoint - currentValue[0, 0]  # Cart position error
        self.integralX += errorX * dt
        derivativeX = (errorX - self.prevErrorX) / dt if dt > 0 else 0.0
        self.prevErrorX = errorX

        # Inner loop for pole angle control
        thetaDesired = self.KpX * errorX + self.KiX * self.integralX + self.KdX * derivativeX  # Desired pole angle based on cart position error

        errorTheta = angle_difference(thetaDesired, currentValue[1, 0])  # Pole angle error
        self.integralTheta += errorTheta * dt
        derivativeTheta = (errorTheta - self.prevErrorTheta) / dt if dt > 0 else 0.0
        self.prevErrorTheta = errorTheta

        # Compute control signal (force)
        controlSignal = self.KpTheta * errorTheta + self.KiTheta * self.integralTheta + self.KdTheta * derivativeTheta
        
        return controlSignal
    


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

def angle_difference(target, current):
    """
    Computes the shortest difference between two angles in radians, returning a value in the range (-pi, pi].
    """
    diff = wrap_to_pi(target - current)
    return diff