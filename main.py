import pygame
import sys
import numpy as np
sys.path.insert(1, './src')
import cart_pole
import controllers
import screen_tools

if __name__ == "__main__":
    pygame.init()

    # Controllers
    piController = controllers.PIDController()

    # CartPoles
    pidCartPole           = cart_pole.CartPole(piController)
    stateFeedbackCartPole = cart_pole.CartPole()
    lqrCartPole           = cart_pole.CartPole()
    mpcCartPole           = cart_pole.CartPole()

    # Canvas
    screen              = pygame.display.set_mode((screen_tools.SCREEN_WIDTH, screen_tools.SCREEN_HEIGHT))
    pidCanvas           = screen_tools.Canvas(screen, screen_tools.PID_CANVAS_POS           , pidCartPole          )
    stateFeedbackCanvas = screen_tools.Canvas(screen, screen_tools.STATE_FEEDBACK_CANVAS_POS, stateFeedbackCartPole)
    lqrCanvas           = screen_tools.Canvas(screen, screen_tools.LQR_CANVAS_POS           , lqrCartPole          )
    mpcCanvas           = screen_tools.Canvas(screen, screen_tools.MPC_CANVAS_POS           , mpcCartPole          )

    # Displays
    pygame.display.set_caption("Cart Pole Control")

    # Slider Set Point
    setPointSlider = screen_tools.SlideBar(screen)

    # Time control
    dt = 0.01  # Time step for the simulation


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEMOTION:
                setPointSlider.handle_event(event)

        screen.fill(screen_tools.colors["white"])  # Clear the screen with white background

        # Draw cart animations first
        pidCanvas.draw_cart()
        stateFeedbackCanvas.draw_cart()
        lqrCanvas.draw_cart()
        mpcCanvas.draw_cart()

        # Fill screen with static elements (titles, labels, etc.)
        screen_tools.draw_static_screen(screen)

        # Draw slider set point
        setPointSlider.draw()


        # Apply controllers and update states
        pidCartPole.apply_controller(setPointSlider.get_set_point(), dt)
        #print(f"PID Cart Pole State: {pidCartPole.get_current_state().flatten()}")


        



        pygame.display.flip()    # Update the display
        pygame.time.delay(10)   # Delay to control frame rate

    pygame.quit()
