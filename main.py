import pygame
import sys
import numpy as np
sys.path.insert(1, './src')
import cart_pole
import controllers
import screen_tools

if __name__ == "__main__":
    pygame.init()

    # CartPoles
    pidCartPole           = screen_tools.CartPole()
    stateFeedbackCartPole = screen_tools.CartPole()
    lqrCartPole           = screen_tools.CartPole()
    mpcCartPole           = screen_tools.CartPole()

    # Canvas
    screen              = pygame.display.set_mode((screen_tools.SCREEN_WIDTH, screen_tools.SCREEN_HEIGHT))
    pidCanvas           = screen_tools.Canvas(screen, screen_tools.PID_CANVAS_POS           , pidCartPole          )
    stateFeedbackCanvas = screen_tools.Canvas(screen, screen_tools.STATE_FEEDBACK_CANVAS_POS, stateFeedbackCartPole)
    lqrCanvas           = screen_tools.Canvas(screen, screen_tools.LQR_CANVAS_POS           , lqrCartPole          )
    mpcCanvas           = screen_tools.Canvas(screen, screen_tools.MPC_CANVAS_POS           , mpcCartPole          )

    # Displays
    pygame.display.set_caption("Cart Pole Control")


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(screen_tools.colors["white"])  # Clear the screen with white background

        # Draw cart animations first
        pidCanvas.draw_cart()
        stateFeedbackCanvas.draw_cart()
        lqrCanvas.draw_cart()
        mpcCanvas.draw_cart()

        screen_tools.draw_static_screen(screen)

        



        pygame.display.flip()    # Update the display

    pygame.quit()
