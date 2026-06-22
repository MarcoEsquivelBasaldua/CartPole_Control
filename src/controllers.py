import numpy as np
from scipy.linalg import solve_continuous_are

print("Cart Pole Controllers Imported")

MAX_FORCE = 50.0 # Newtons

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

        #print(f"PID Control Signal: {controlSignal:.4f}, Cart Position Error: {errorX:.4f}, Pole Angle Error: {errorTheta:.4f}")

        return controlSignal, errorTheta, errorX
    
    def reset(self):
        """
        Resets the internal state of the PID controller.
        """
        self.integralX      = 0.0
        self.integralTheta  = 0.0
        self.prevErrorX     = 0.0
        self.prevErrorTheta = 0.0
    

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

        # Errors
        errorX = setpoint - currentState[0, 0] # Cart position error
        errorTheta = angle_difference(0.0, currentState[1, 0])  # Pole angle error (desired angle is 0 for upright)

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

        #print(f"LQR Control Signal: {controlSignal[0, 0]:.4f}, Cart Position Error: {x[0, 0]:.4f}, Pole Angle Error: {x[1, 0]:.4f}")

        return controlSignal[0, 0], errorTheta, errorX
    
    def reset(self):
        """
        Resets the internal state of the LQR controller if needed. For a standard LQR controller, there may not be any internal state to reset, but this method is included for consistency and future extensibility.
        """
        pass  # No internal state to reset for standard LQR, but this can be implemented if needed in the future


class fuzzyLogicController:
    def __init__(self):

        # To integrate bell membership functions
        self.deltaInt = 0.1
        samples  = int((2*MAX_FORCE) / self.deltaInt)

        self.horVectorforce = np.linspace(-MAX_FORCE, MAX_FORCE, samples)

        # Bell functions constants
        self.thetaBellConsts = {'a': 5, 'b': 2, 'c': 20}

    def compute_control(self, setpoint: float, currentState: np.ndarray, dt: float) -> float:
        theta = currentState[1, 0]  # Pole angle
        errorX = setpoint - currentState[0, 0] # Cart position error
        errorTheta = angle_difference(0.0, theta)  # Pole angle error (desired angle is 0 for upright)

        # Compute the degree of membership for the pole angle error
        thetaMembership = self.__degree_of_membership(theta, scalling=10.0)
        #print(f"Pole Angle Error: {errorTheta:.4f}, Positive Membership: {posThetaMembership:.4f}, Negative Membership: {negThetaMembership:.4f}")

        self.__combined_force(thetaMembership)

        return 0.0, errorTheta, errorX

    def __degree_of_membership(self, signal:float, scalling = 1.0) -> tuple:
        """
        Computes the degree of membership for a given signal using a sigmoid function.
        Parameters:
            signal (float): The input signal for which to compute the membership.
            scalling (float): A scaling factor to adjust the steepness of the sigmoid function.
        Returns:
            tuple: A tuple containing the positive and negative membership values.
        """
        positveMembership  = 1.0 / (1.0 + np.e**(-scalling * signal))
        negativeMembership = 1.0 - positveMembership

        return positveMembership, negativeMembership

    def __bell_membership_function(self, x, params, negative=False):
        """
        Bell-shaped membership function for fuzzy logic.
        Parameters:
            x (float): The input value.
            params (dict): A dictionary containing the parameters of the bell curve, including 'a', 'b', and 'c'.
                a (float): The width of the bell curve.
                b (float): The slope of the bell curve.
                c (float): The center of the bell curve.

        Returns:
            float: The membership value for the input x.
        """
        a = params['a']
        b = params['b']
        c = params['c'] if not negative else -params['c']  # Center can be negative for negative error

        return 1 / (1 + abs((x - c) / a) ** (2 * b))
    
    def __combined_force(self, thetaDegs):

        # Bell equiation functions
        posThetaBell = self.__bell_membership_function(self.horVectorforce, self.thetaBellConsts)
        negThetaBell = self.__bell_membership_function(self.horVectorforce, self.thetaBellConsts, negative=True)

        # Positive memberships
        posThetaDeg = thetaDegs[0]
        posThetaVals = np.minimum(posThetaBell, posThetaDeg)

        # Negative memberships
        negThetaDeg = thetaDegs[1]
        negThetaVals = np.minimum(negThetaBell, negThetaDeg)

        # Combined memberships
        thetaVals = np.maximum(negThetaVals, posThetaVals)

        # Force as x centroid coordinate
        force = self.__hor_centroid(thetaVals)

        print(force)
        
        
        import matplotlib.pyplot as plt
        plt.figure()
        #plt.plot(self.horVectorforce, posThetaVals)
        #plt.plot(self.horVectorforce, negThetaVals)
        plt.plot(self.horVectorforce, thetaVals)
        plt.title("Bell Membership Function")
        plt.xlabel("Force")
        plt.ylabel("Membership")
        plt.grid(True)
        plt.show()


    def __hor_centroid(self, yVals):
        totalArea = np.sum(yVals) * self.deltaInt
        xtimesY   = self.horVectorforce * yVals
        xCentroid = (1/totalArea) * np.sum(xtimesY) * self.deltaInt

        return xCentroid




        

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
