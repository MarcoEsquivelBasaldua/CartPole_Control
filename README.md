# CartPole_Control

## System Description
The cart-pole system is a classic problem in control theory and robotics. It consists of a cart that can move horizontally along a track and a pole (or pendulum) that is attached to the cart. The objective is to apply a force to the cart in such a way that the pole remains balanced in the upright position.

The cart-pole system is composed of two main components:
1. The cart, $c$, which has a mass $m_c$ and can move horizontally along a track. The cart is controlled by applying a force $F$ to it.
2. The pole, $p$, which can rotate without friction around the cart center of mass. It has a mass $m_p$ and a length $L$. The angle of the pole with respect to the vertical is denoted by $\theta$. The system is influenced by gravity, which acts on the pole with an acceleration $g$.

wheels on the cart are not part of the system. They illustrate the absence of friction between the cart and the ground.

![Cart-Pole System](./images/cartPole.png)

## Equations of Motion
The equations of motion for the cart-pole system can be derived using Newton's second law or Lagrangian mechanics. The state of the system can be described by the position of the cart $x$ and the angle of the pole $\theta$. The equations of motion are given by:

$$
\begin{aligned}
(m_c + m_p)\ddot{x} +\frac{m_pL}{2} \left(\cos(\theta)\ddot{\theta} - \sin(\theta)\dot{\theta}^2\right) &= F \\
\cos(\theta)\ddot{x} + \frac{2L}{3}\ddot{\theta} - g \sin(\theta) &= 0
\end{aligned}
$$

where $F$ is the force applied to the cart, $g$ is the acceleration due to gravity, $\dot{x}$ is the velocity of the cart, and $\dot{\theta}$ is the angular velocity of the pole.

## State Space Representation
To analyze and design controllers for the cart-pole system, we can represent it in state-space form. We define the state vector $\mathbf{x}$ as:

$$
\mathbf{x} = \left[\begin{matrix} x \\ 
\theta \\ 
\dot{x} \\ 
\dot{\theta}
\end{matrix}\right]
$$

The state-space representation of the system can be obtain from the formulation of the equations of motion. The system can be expressed in the form:

$$
    \mathbf{M}(x, \theta)\left[\begin{matrix} \ddot{x} \\ 
    \ddot{\theta} \end{matrix}\right] + \mathbf{C}(x, \theta, \dot{x}, \dot{\theta})\left[\begin{matrix} \dot{x} \\ 
    \dot{\theta} \end{matrix}\right] + \mathbf{G}(x, \theta) = \mathbf{B}u
$$

Where $\mathbf{M}(\mathbf{x})$ is the mass matrix, $\mathbf{C}(\mathbf{x}, \dot{\mathbf{x}})$ is the Coriolis and centrifugal matrix, $\mathbf{G}(\mathbf{x})$ is the gravity vector, $\mathbf{B}$ is the input matrix, and $u$ is the control input (force applied to the cart).

Replacing $\mathbf{M}(x, \theta)$, $\mathbf{C}(x, \theta, \dot{x}, \dot{\theta})$, $\mathbf{G}(x, \theta)$ and $\mathbf{B}$, we have

$$
    \begin{bmatrix} m_c + m_p & \frac{m_pL}{2}\cos(\theta) \\ 
    \cos(\theta) & \frac{2L}{3} \end{bmatrix}\left[\begin{matrix} \ddot{x} \\ 
    \ddot{\theta} \end{matrix}\right] + \begin{bmatrix} 0 & -\frac{m_pL}{2}\sin(\theta)\dot{\theta} \\ 
    0 & 0 \end{bmatrix}\left[\begin{matrix} \dot{x} \\ 
    \dot{\theta} \end{matrix}\right] + \begin{bmatrix} 0 \\ 
    -g\sin(\theta) \end{bmatrix} = \begin{bmatrix} 1 \\ 
    0 \end{bmatrix}u
$$

with 

$$
u = F
$$

The state-space representation is then given by:

$$
    \dot{\mathbf{x}} = \begin{bmatrix} \dot{x} \\ 
    \dot{\theta} \\
    -\mathbf{M}^{-1}(x, \theta)\left( \mathbf{C}(x, \theta, \dot{x}, \dot{\theta})\left[\begin{matrix} \dot{x} \\ 
    \dot{\theta} \end{matrix}\right] + \mathbf{G}(x, \theta) \right) \end{bmatrix} + \begin{bmatrix} 0 \\ 
    0 \\
    \mathbf{M}^{-1}(x, \theta)\mathbf{B} \end{bmatrix}u
$$

Since we only see the position of the cart and the angle of the pole, we can define the output vector $\mathbf{y}$ as:

$$
\mathbf{y} = \left[\begin{matrix} 1 & 0 & 0 & 0 \\ 
0 & 1 & 0 & 0 \end{matrix}\right]\mathbf{x}
$$

## Control Objective
The control objective for the cart-pole system is to design a controller that can stabilize the pole in the upright position ($\theta = 0$) while keeping the cart at a desired position ($x = x_{desired}$). This can be achieved by applying an appropriate force $F$ to the cart based on the current state of the system.

In this project, we will explore various control strategies, such as PID control, Linear Quadratic Regulator (LQR), Lyapunov direct method, and model predictive control, to achieve the desired control objective. Each of these methods will be implemented and shown side by side to compare their performance in stabilizing the cart-pole system.

## PID Control
PID (Proportional-Integral-Derivative) control is a widely used control strategy that combines three terms to compute the control input: the proportional term, which is based on the current error; the integral term, which is based on the accumulated error over time; and the derivative term, which is based on the rate of change of the error.

The current system is an underactuated system, meaning that we have fewer control inputs than the number of states. In this case, we have only one control input (the force $F$ applied to the cart) to control both the position of the cart and the angle of the pole. This makes it challenging to design a PID controller that can effectively stabilize the system.

The control in this case is achieved via a cascade control structure, where we have an inner loop that controls the angle of the pole and an outer loop that controls the position of the cart. The inner loop is designed to stabilize the pole in the upright position, while the outer loop is designed to keep the cart at the desired position.

The PID controller for the inner loop is designed as follows:

$$
    F = - K_{p,\theta} {\theta}_e(t) - K_{i,\theta} \int {\theta}_e(t) dt - K_{d,\theta} \dot{\theta}_e(t)
$$

Where 

$$
    {\theta}_e(t) = \theta_r(t) - \theta(t)
$$

The PID controller for the outer loop is designed as follows:

$$
    \theta_r(t) = K_{p,x} x_e(t) + K_{i,x} \int x_e(t) dt + K_{d,x} \dot{x}_e(t)
$$

Where

$$
    x_e(t) = x_r(t) - x(t)
$$

## LQR Control
The Linear Quadratic Regulator (LQR) is an optimal control strategy that computes the control input by minimizing a quadratic cost function that penalizes deviations from the desired state and the control effort. The LQR controller is designed based on the linearized dynamics of the system around the equilibrium point. The cost function for the LQR controller is given by:

$$
    J = \int_0^\infty \left( \mathbf{x}^T Q \mathbf{x} + u^T R u \right) dt
$$

Where $Q$ is the state cost matrix that penalizes deviations from the desired state, and $R$ is the control cost matrix that penalizes the control effort. The LQR controller computes the optimal control input as:

$$
    u = -K \mathbf{x}
$$

Where $K$ is the LQR gain matrix that can be computed by solving the Continuous-time Algebraic Riccati Equation (CARE):

$$
    A^T P + P A - P B R^{-1} B^T P + Q = 0
$$

Where $P$ is the optimal state cost matrix. The LQR controller is designed to stabilize the system while minimizing the cost function, which leads to a trade-off between performance and control effort.
    