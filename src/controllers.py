import numpy as np
from scipy.linalg import solve_continuous_are

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

    def compute_control(self, setpoint: float, currentState: np.ndarray, dt: float, A: np.ndarray = None, B: np.ndarray = None) -> float:
        """
        Computes the control signal (force) based on the current state of the system and a desired setpoint.
        Parameters:
            setpoint (float): The desired cart position.
            currentState (np.ndarray): The current state of the system, where currentState[0, 0] is the cart position and currentState[1, 0] is the pole angle.
            dt (float): The time step for the simulation.
            A (np.ndarray): The A matrix for the linearized system.
            B (np.ndarray): The B matrix for the linearized system.
        Returns:            
            float: The computed control signal (force) to be applied to the cart.
        """
        # Outer loop for cart position control
        errorX = setpoint - currentState[0, 0] # Cart position error
        self.integralX += errorX * dt
        derivativeX = (errorX - self.prevErrorX) / dt if dt > 0 else 0.0
        self.prevErrorX = errorX

        # Inner loop for pole angle control
        thetaDesired = self.KpX * errorX + self.KiX * self.integralX + self.KdX * derivativeX  # Desired pole angle based on cart position error

        errorTheta = angle_difference(thetaDesired, currentState[1, 0])  # Pole angle error
        self.integralTheta += errorTheta * dt
        derivativeTheta = wrap_to_pi(errorTheta - self.prevErrorTheta) / dt if dt > 0 else 0.0
        self.prevErrorTheta = errorTheta

        # Compute control signal (force)
        controlSignal = - self.KpTheta * errorTheta - self.KiTheta * self.integralTheta - self.KdTheta * derivativeTheta

        return controlSignal, errorTheta, errorX
    

class LQRController:
    """
    A placeholder for the LQR controller. The actual implementation will compute the optimal control signal based on the linearized system dynamics and a cost function.
    """
    def __init__(self):
        """
        Initializes the LQR controller with default cost matrices.
        """
        self.Q_x        = 20.0
        self.Q_theta    = 200.0
        self.Q_xdot     = 1.0
        self.Q_thetadot = 10.0
        self.R          = 1.0

    def compute_control(self, setpoint: float, currentState: np.ndarray, dt: float, A: np.ndarray, B: np.ndarray) -> float:
        """Computes the control signal (force) based on the current state of the system and a desired setpoint using LQR control.
        Parameters:
            setpoint (float): The desired cart position.
            currentState (np.ndarray): The current state of the system, where currentState[0, 0] is the cart position and currentState[1, 0] is the pole angle.
            dt (float): The time step for the simulation.
            A (np.ndarray): The A matrix for the linearized system.
            B (np.ndarray): The B matrix for the linearized system.

        Returns:
            float: The computed control signal (force) to be applied to the cart.
        """

        # Define the state vector
        x = currentState

        # Add state shift to account for the setpoint (desired cart position)
        x[0, 0] -= setpoint  # Shift the cart position to be relative to the setpoint

        # Define the cost matrices for LQR
        Q = np.diag([self.Q_x,
                     self.Q_theta,
                     self.Q_xdot,
                     self.Q_thetadot])  # State cost matrix (penalize pole angle more than cart position)
        R = np.array([[self.R]])        # Control cost matrix

        # Solve the Continuous-time Algebraic Riccati Equation (CARE) to find the optimal state cost matrix P
        P = solve_continuous_are(A, B, Q, R)

        # Compute the LQR gain matrix K
        K = np.linalg.inv(R) @ B.T @ P

        # Compute the control signal using the LQR gain
        controlSignal = -K @ x

        return controlSignal[0, 0], angle_difference(0.0, currentState[1, 0]), setpoint - currentState[0, 0]
        

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
