# CartPole_Control

## System Description
The cart-pole system is a classic problem in control theory and robotics. It consists of a cart that can move horizontally along a track and a pole (or pendulum) that is attached to the cart. The objective is to apply a force to the cart in such a way that the pole remains balanced in the upright position.

The cart-pole system is composed of two main components:
1. The cart, \(c\), which has a mass \( m_c \) and can move horizontally along a track. The cart is controlled by applying a force \( F \) to it.
2. The pole, \(p\), which can rotate without friction around the cart center of mass. It has a mass \( m_p \) and a length \( L \). The angle of the pole with respect to the vertical is denoted by \( \theta \). The system is influenced by gravity, which acts on the pole with an acceleration \( g \).

wheels on the cart are not part of the system. They illustrate the absence of friction between the cart and the ground.

![Cart-Pole System](./images/cartPole.png)

