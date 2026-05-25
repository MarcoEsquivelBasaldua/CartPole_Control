import numpy as np

class PIDController:
    def __init__(self):
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
        setpointVect = np.array([[setpoint],  # x cart position setpoint
                                 [0.0]])      # pole angle setpoint (upright)
        
        currentValueVect = np.array([[currentValue[0, 0]],  # x cart position
                                    [currentValue[1, 0]]])  # pole angle
        
        poleVel = currentValue[3, 0]  # Pole angular velocity (radians/s)

        # Outer loop for cart position control
        errorX = setpoint - currentValue[0, 0] # Cart position error
        self.integralX += errorX * dt
        derivativeX = (errorX - self.prevErrorX) / dt if dt > 0 else 0.0
        self.prevErrorX = errorX

        # Inner loop for pole angle control
        thetaDesired = self.KpX * errorX + self.KiX * self.integralX + self.KdX * derivativeX  # Desired pole angle based on cart position error
        #thetaDesired = 0.0  # For now, we'll keep the pole upright

        errorTheta = angle_difference(thetaDesired, currentValue[1, 0])  # Pole angle error
        self.integralTheta += errorTheta * dt
        derivativeTheta = wrap_to_pi(errorTheta - self.prevErrorTheta) / dt if dt > 0 else 0.0
        self.prevErrorTheta = errorTheta

        # Compute control signal (force)
        controlSignal = - self.KpTheta * errorTheta - self.KiTheta * self.integralTheta - self.KdTheta * derivativeTheta

        #controlSignal = thetaDesired * 100.0  # Scale the desired angle to get a control signal (this is a simple heuristic)
        

        #print(currentValue[1, 0])
        #print("control ",controlSignal)
        #print(currentValue[3, 0]) # Pole angular velocity (radians/s)
        #print(errorX)
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