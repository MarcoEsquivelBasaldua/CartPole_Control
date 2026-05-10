import numpy as np

print("Cart Pole Control Imported")

# Initial conditions and limits
INITIAL_CART_X         = 0.0
INITIAL_CART_X_VEL     = 0.0
INITIAL_CART_X_ACC     = 0.0
INITIAL_POLE_ANGLE     = np.pi
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
    def __init__(self):
        self.cartMass    = CART_MASS
        self.cartX       = INITIAL_CART_X
        self.cartXd      = INITIAL_CART_X_VEL
        self.cartXdd     = INITIAL_CART_X_ACC
        self.height      = CART_HEIGHT
        self.length      = CART_LENGTH
        self.wheelRadius = WHEEL_RADIUS

        self.poleMass    = POLE_MASS
        self.poleLength  = POLE_LENGTH
        self.poleAngle   = INITIAL_POLE_ANGLE
        self.poleAngled  = INITIAL_POLE_ANGLE_VEL
        self.poleAngledd = INITIAL_POLE_ANGLE_ACC

        