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
    pidController   = controllers.PIDController()
    lqrController   = controllers.LQRController()
    fuzzyController = controllers.fuzzyLogicController()

    # CartPoles
    pidCartPole   = cart_pole.CartPole(pidController)
    lqrCartPole   = cart_pole.CartPole(lqrController)
    fuzzyCartPole = cart_pole.CartPole(fuzzyController)
    mpcCartPole = cart_pole.CartPole()

    # Canvas
    screen    = pygame.display.set_mode((screen_tools.SCREEN_WIDTH, screen_tools.SCREEN_HEIGHT))
    pidCanvas = screen_tools.Canvas(screen, screen_tools.PID_CANVAS_POS, pidCartPole)
    lqrCanvas = screen_tools.Canvas(screen, screen_tools.LQR_CANVAS_POS, lqrCartPole)
    fuzzyCanvas = screen_tools.Canvas(screen, screen_tools.FUZZY_CANVAS_POS, fuzzyCartPole)
    mpcCanvas = screen_tools.Canvas(screen, screen_tools.MPC_CANVAS_POS, mpcCartPole)

    # Displays
    pygame.display.set_caption("Cart Pole Control")

    # Slider Set Point
    setPointSlider = screen_tools.SlideBar(screen)

    # Reset Button
    BUTTON_WIDTH_SMALL = 100
    BUTTON_HEIGHT_SMALL = 50
    RESET_BUTTON = screen_tools.Button(120, 70, BUTTON_WIDTH_SMALL, BUTTON_HEIGHT_SMALL, 'RESET', screen)

    # Time control
    dt = 0.01  # Time step for the simulation


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                wasMousePresed = True
                RESET_BUTTON.handle_event(event)
            elif event.type == pygame.MOUSEMOTION:
                setPointSlider.handle_event(event)

        screen.fill(screen_tools.colors["white"])  # Clear the screen with white background

        # Draw cart animations first
        pidCanvas.draw_cart()
        lqrCanvas.draw_cart()
        fuzzyCanvas.draw_cart()
        mpcCanvas.draw_cart()

        # Fill screen with static elements (titles, labels, etc.)
        screen_tools.draw_static_screen(screen)

        # Draw slider set point
        setPointSlider.draw()

        # Draw reset button
        RESET_BUTTON.draw()


        # Apply controllers and update states

        # PID Controller
        pidCartPole.apply_controller(setPointSlider.get_set_point(), dt)
        pidCanvas.plot_angle_error(pidCartPole.get_angle_error_history())
        pidCanvas.plot_displacement_error(pidCartPole.get_displacement_error_history())

        # LQR Controller
        lqrCartPole.apply_controller(setPointSlider.get_set_point(), dt, linearize=True)
        lqrCanvas.plot_angle_error(lqrCartPole.get_angle_error_history())
        lqrCanvas.plot_displacement_error(lqrCartPole.get_displacement_error_history())

        # Fuzzy Controller
        fuzzyCartPole.apply_controller(setPointSlider.get_set_point(), dt)
        fuzzyCanvas.plot_angle_error(fuzzyCartPole.get_angle_error_history())
        fuzzyCanvas.plot_displacement_error(fuzzyCartPole.get_displacement_error_history())

        # Reset simulations
        if RESET_BUTTON.was_button_pressed():
            setPointSlider.reset()  # Reset slider to initial position
            pidCartPole.reset()
            lqrCartPole.reset()
            fuzzyCartPole.reset()
            #mpcCartPole.reset()

            RESET_BUTTON.reset()  # Reset button state


        



        wasMousePresed = False
        pygame.display.flip()    # Update the display
        pygame.time.delay(10)   # Delay to control frame rate

    pygame.quit()
