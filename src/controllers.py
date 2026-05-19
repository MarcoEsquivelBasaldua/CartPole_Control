import numpy as np

class PIDController:
    def __init__(self):
        self.KpX = 0.08
        self.KiX = 0.001
        self.KdX = 0.02

        self.KpTheta = 80.0
        self.KiTheta = 0.0
        self.KdTheta = 5.5

        self.integralX      = 0.0
        self.integralTheta  = 0.0
        self.prevErrorX     = 0.0
        self.prevErrorTheta = 0.0

    def compute_control(self, setpoint: float, currentValue: np.ndarray, dt: float) -> float:
        setpointVect = np.array([[setpoint],  # x cart position setpoint
                                 [0.0]])      # pole angle setpoint (upright)
        
        currentValueVect = np.array([[currentValue[0, 0]],  # x cart position
                                    [currentValue[1, 0]]])  # pole angle

        # Outer loop for cart position control
        errorX = currentValue[0, 0] - setpoint # Cart position error
        self.integralX += errorX * dt
        derivativeX = (errorX - self.prevErrorX) / dt if dt > 0 else 0.0
        cartVelocity = currentValue[2, 0]  # Cart velocity (m/s)
        self.prevErrorX = errorX

        # Inner loop for pole angle control
        thetaDesired = - self.KpX * errorX - self.KiX * self.integralX - self.KdX * cartVelocity  # Desired pole angle based on cart position error

        errorTheta = angle_difference(currentValue[1, 0], thetaDesired)  # Pole angle error
        self.integralTheta += errorTheta * dt
        poleAngularVelocity = currentValue[3, 0]  # Pole angular velocity (radians/s)
        self.prevErrorTheta = errorTheta

        # Compute control signal (force)
        controlSignal = self.KpTheta * errorTheta - self.KiTheta * self.integralTheta + self.KdTheta * poleAngularVelocity
        

        #print(currentValue[1, 0])
        print("control ",controlSignal)
        #print(currentValue[3, 0]) # Pole angular velocity (radians/s)
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

def angle_difference(current, target):
    """
    Computes the shortest difference between two angles in radians, returning a value in the range (-pi, pi].
    """
    diff = wrap_to_pi(current - target)
    return diff

def angle_difference2(target, current, vel):
    piOver2 = np.pi / 2.0
    
    if current >= piOver2:
        if vel >= 0:
            diff = 2 *np.pi - current
        else:
            diff = angle_difference(current, target)
    elif current <= -piOver2:
        if vel <= 0:
            diff = -2 * np.pi + current
        else:
            diff = angle_difference(current, target)
    else:
        diff = angle_difference(target, current)
    return diff