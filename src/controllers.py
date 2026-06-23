import numpy as np
from scipy.linalg import solve_continuous_are

print("Cart Pole Controllers Imported")

MAX_FORCE = 100.0 # Newtons

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
        deltaInt = 0.1
        samples  = int((2*MAX_FORCE) / deltaInt)

        self.horVectorforce = np.linspace(-MAX_FORCE, MAX_FORCE, samples)

        # Bell functions constants
        self.thetaBellConsts    = {'a': 25, 'b': 5, 'c': 60}
        self.thetaDotBellConsts = {'a': 8, 'b': 5, 'c': 80}
        self.xErrorBellConsts   = {'a': 12, 'b': 5, 'c': 20 }
        self.xDotBellConsts     = {'a': 10, 'b': 5, 'c': 30}

    def reset(self):
        pass

    def compute_control(self, setpoint: float, currentState: np.ndarray, dt: float) -> float:
        xPos, theta, xPosDot, thetaDot  = currentState.flatten()
        
        errorX     = setpoint - xPos               # Cart position error
        errorTheta = angle_difference(0.0, theta)  # Pole angle error (desired angle is 0 for upright)

        # Compute the degree of membership every state entry
        thetaMembership    = self.__degree_of_membership(theta   , scalling=50.0)
        thetaDotMembership = self.__degree_of_membership(thetaDot, scalling=20.0 )
        xErrorMembership   = self.__degree_of_membership(errorX  , scalling=-20.0)
        xDotMembership     = self.__degree_of_membership(xPosDot , scalling=10.0 )

        force = self.__combined_force(thetaMembership, thetaDotMembership, xErrorMembership, xDotMembership)

        return force, errorTheta, errorX

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

    def __bell_membership_function(self, params, negative=False):
        """
        Bell-shaped membership function for fuzzy logic.
        Parameters:
            params (dict): A dictionary containing the parameters of the bell curve, including 'a', 'b', and 'c'.
                a (float): The width of the bell curve.
                b (float): The slope of the bell curve.
                c (float): The center of the bell curve.

        Returns:
            float: The membership value for the input x.
        """
        x = self.horVectorforce
        a = params['a']
        b = params['b']
        c = params['c'] if not negative else -params['c']  # Center can be negative for negative error

        return 1 / (1 + abs((x - c) / a) ** (2 * b))

    def __combined_force(self, thetaDegs, thetaDotDegs, xErrorDegs, xDotDegs):

        # Bell equiation functions
        posThetaBell    = self.__bell_membership_function(self.thetaBellConsts)
        negThetaBell    = self.__bell_membership_function(self.thetaBellConsts, negative=True)
        posThetaDotBell = self.__bell_membership_function(self.thetaDotBellConsts)
        negThetaDotBell = self.__bell_membership_function(self.thetaDotBellConsts, negative=True)
        posxErrorBell   = self.__bell_membership_function(self.xErrorBellConsts)
        negxErrorBell   = self.__bell_membership_function(self.xErrorBellConsts, negative=True)
        posxDotBell     = self.__bell_membership_function(self.xDotBellConsts)
        negxDotBell     = self.__bell_membership_function(self.xDotBellConsts, negative=True)

        # Positive memberships
        posThetaVals    = np.minimum(posThetaBell, thetaDegs[0])
        posThetaDotVals = np.minimum(posThetaDotBell, thetaDotDegs[0])
        posxErrorVals   = np.minimum(posxErrorBell, xErrorDegs[0])
        posxDotVals     = np.minimum(posxDotBell, xDotDegs[0])

        # Negative memberships
        negThetaVals    = np.minimum(negThetaBell, thetaDegs[1])
        negThetaDotVals = np.minimum(negThetaDotBell, thetaDotDegs[1])
        negxErrorVals   = np.minimum(negxErrorBell, xErrorDegs[1])
        negxDotVals     = np.minimum(negxDotBell, xDotDegs[1])

        # Combined memberships
        thetaVals    = np.maximum(negThetaVals   , posThetaVals)
        thetaDotVals = np.maximum(negThetaDotVals, posThetaDotVals)
        xErrorVals   = np.maximum(negxErrorVals, posxErrorVals)
        xDotVals     = np.maximum(negxDotVals, posxDotVals)

        combinedVals = np.maximum(thetaVals, np.maximum(thetaDotVals, np.maximum(xErrorVals, xDotVals)))

        #combinedVals = thetaVals
        #combinedVals = np.maximum(thetaVals, xErrorVals)
        #combinedVals = np.maximum(thetaVals, np.maximum(thetaDotVals, xErrorVals))
        #combinedVals = np.maximum(thetaVals, thetaDotVals)
        #combinedVals = np.maximum(thetaVals,xErrorVals)

        # Force as x centroid coordinate
        force = self.__hor_centroid(combinedVals)
        print(force)

        return force

        
        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(self.horVectorforce, negThetaDotBell)
        plt.plot(self.horVectorforce, posThetaDotBell)
        plt.plot(self.horVectorforce, combinedVals)
        plt.title("Bell Membership Function")
        plt.xlabel("Force")
        plt.ylabel("Membership")
        plt.grid(True)
        plt.show()

        return force


    def __hor_centroid(self, yVals):
        totalArea = np.sum(yVals)
        xtimesY   = self.horVectorforce * yVals
        xCentroid = (1/totalArea) * np.sum(xtimesY)

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
