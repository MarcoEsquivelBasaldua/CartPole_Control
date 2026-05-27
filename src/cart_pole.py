import numpy as np

from controllers import wrap_to_pi

print("Cart Pole Class Imported")

# Initial conditions and limits
INITIAL_CART_X         = 0.0
INITIAL_CART_X_VEL     = 0.0
INITIAL_CART_X_ACC     = 0.0
INITIAL_POLE_ANGLE     = np.pi / 4.0  # 30 degrees from vertical
INITIAL_POLE_ANGLE_VEL = 0.0
INITIAL_POLE_ANGLE_ACC = 0.0
MAX_CART_DISPLACEMENT  = 5.0  # meters

# Physical parameters
CART_MASS    = 1.0  # kg
POLE_MASS    = 0.5  # kg
CART_HEIGHT  = 0.6  # meters
CART_LENGTH  = CART_HEIGHT * 1.618
WHEEL_RADIUS = 0.1  # meters
POLE_LENGTH  = 1.0  # meters

class CartPole:
    def __init__(self, controller=None):
        """
        Initializes the CartPole system with default parameters and initial conditions.
        """
        self.controller  = controller
        self.cartMass    = CART_MASS
        self.cartX       = INITIAL_CART_X
        self.cartXdot    = INITIAL_CART_X_VEL
        self.cartXddot   = INITIAL_CART_X_ACC
        self.height      = CART_HEIGHT
        self.length      = CART_LENGTH
        self.wheelRadius = WHEEL_RADIUS

        self.poleMass      = POLE_MASS
        self.poleLength    = POLE_LENGTH
        self.poleAngle     = INITIAL_POLE_ANGLE
        self.poleAngledot  = INITIAL_POLE_ANGLE_VEL
        self.poleAngleddot = INITIAL_POLE_ANGLE_ACC

        self.forceHistory             = []
        self.angleErrorHistory        = []
        self.displacementErrorHistory = []

    def reset(self):
        """
        Resets the CartPole system to its initial conditions.
        """
        self.cartX       = INITIAL_CART_X
        self.cartXdot    = INITIAL_CART_X_VEL
        self.cartXddot   = INITIAL_CART_X_ACC

        self.poleAngle     = INITIAL_POLE_ANGLE
        self.poleAngledot  = INITIAL_POLE_ANGLE_VEL
        self.poleAngleddot = INITIAL_POLE_ANGLE_ACC

    def get_current_state(self):
        """
        Returns the current state of the CartPole system as a tuple.
        """
        state = np.array([[self.cartX], 
                          [self.poleAngle], 
                          [self.cartXdot],
                          [self.poleAngledot]])
        return state
    
    def __equations_of_motion(self, force: float):
        """
        Gets the equations of motion for the CartPole system, returning the mass matrix, the Coriolis and gravity vector, and the input matrix.
        """
        theta     = self.poleAngle
        theta_dot = self.poleAngledot
        m_c       = self.cartMass
        m_p       = self.poleMass
        L         = self.poleLength
        g         = 9.81

        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        mpl_2     = (m_p * L) / 2.0

        # Mass matrix
        M = np.array([[m_c + m_p, mpl_2 * cos_theta],
                      [cos_theta, 2.0 * L / 3.0]])
        
        # Coriolis and gravity vector
        C = np.array([[-mpl_2 * theta_dot**2 * sin_theta],
                      [-g * sin_theta]])
        
        # Input matrix
        B = np.array([[force],
                      [0.0]])
        
        return M, C, B
    
    def __update_state(self, force: float, dt: float):
        """
        Updates the state of the CartPole system based on the applied force and time step.
        """
        M, C, B = self.__equations_of_motion(force)

        # Create the State space representation
        M_inv    = np.linalg.inv(M)
        stateDot = M_inv @ (B - C)
        
        # Solve the state using Euler's method
        self.cartXdot     += stateDot[0, 0] * dt
        self.poleAngledot += stateDot[1, 0] * dt
        self.cartX        += self.cartXdot     * dt
        self.poleAngle    += self.poleAngledot * dt

        self.poleAngle = wrap_to_pi(self.poleAngle)  # Ensure pole angle stays within (-pi, pi]

        #print(f"Cart X: {self.cartX:.2f}, Cart Xdot: {self.cartXdot:.2f}, Pole Angle: {np.degrees(self.poleAngle):.2f} degrees, Pole Angledot: {np.degrees(self.poleAngledot):.2f} degrees/s")

    def apply_controller(self, set_point: float, dt: float):
        """
        Applies the controller to compute the force based on the current state and set point, then updates the state.
        """
        if self.controller is not None:
            force, errorTheta, errorX = self.controller.compute_control(set_point, self.get_current_state(), dt)
            self.__update_state(force, dt)
        else:
            constant_force = 1.0  # No control input
            self.__update_state(constant_force, dt)

        # Save force history for plotting
        self.forceHistory.append(force)
        self.forceHistory = self.forceHistory[-100:]  # Keep only the last 100 entries for plotting
        
        self.angleErrorHistory.append(errorTheta)  # Desired angle is 0 (upright)
        self.angleErrorHistory = self.angleErrorHistory[-100:]  # Keep only the last 100 entries for plotting

        self.displacementErrorHistory.append(errorX)  # Track displacement error
        self.displacementErrorHistory = self.displacementErrorHistory[-100:]  # Keep only the last 100 entries for plotting

    def get_force_history(self):
        """
        Returns the history of applied forces as a list of tuples (time, force).
        """
        return self.forceHistory

    def get_angle_error_history(self):
        """
        Returns the history of angle errors as a list of tuples (time, angle error).
        """
        return self.angleErrorHistory

    def get_displacement_error_history(self):
        """
        Returns the history of displacement errors as a list of tuples (time, displacement error).
        """
        return self.displacementErrorHistory