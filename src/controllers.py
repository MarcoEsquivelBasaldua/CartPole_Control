import numpy as np

print("Cart Pole Controllers Imported")

class PIDController:
    """
    A PID controller for the Cart Pole system. It computes the control signal (force) based on the current state of the system and a desired setpoint.
    """
    def __init__(self):
        """
        Initializes the PID controller with default gains and internal state.
        """
        self.KpX = 0.15
        self.KiX = 0.001
        self.KdX = 0.05

        self.KpTheta = 70.0
        self.KiTheta = 1.0
        self.KdTheta = 10.0

        self.integralX      = 0.0
        self.integralTheta  = 0.0
        self.prevErrorX     = 0.0
        self.prevErrorTheta = 0.0

    def compute_control(self, setpoint: float, currentValue: np.ndarray, dt: float) -> float:
        """
        Computes the control signal (force) based on the current state of the system and a desired setpoint.
        Parameters:
            setpoint (float): The desired cart position.
            currentValue (np.ndarray): The current state of the system, where currentValue[0, 0] is the cart position and currentValue[1, 0] is the pole angle.
            dt (float): The time step for the simulation.
        Returns:            
            float: The computed control signal (force) to be applied to the cart.
        """
        # Outer loop for cart position control
        errorX = setpoint - currentValue[0, 0] # Cart position error
        self.integralX += errorX * dt
        derivativeX = (errorX - self.prevErrorX) / dt if dt > 0 else 0.0
        self.prevErrorX = errorX

        # Inner loop for pole angle control
        thetaDesired = self.KpX * errorX + self.KiX * self.integralX + self.KdX * derivativeX  # Desired pole angle based on cart position error

        errorTheta = angle_difference(thetaDesired, currentValue[1, 0])  # Pole angle error
        self.integralTheta += errorTheta * dt
        derivativeTheta = wrap_to_pi(errorTheta - self.prevErrorTheta) / dt if dt > 0 else 0.0
        self.prevErrorTheta = errorTheta

        # Compute control signal (force)
        controlSignal = - self.KpTheta * errorTheta - self.KiTheta * self.integralTheta - self.KdTheta * derivativeTheta

        return controlSignal, errorTheta, errorX
    


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
