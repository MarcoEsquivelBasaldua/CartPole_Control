import pygame
import sys
import numpy as np
sys.path.insert(1, './src')
import cart_pole
import controllers
import screen_tools

if __name__ == "__main__":
    pygame.init()

    # Environment sizes
    SCREEN_HEIGHT = 1000
    SCREEN_WIDTH  = 1620

    # Positions
    TITLE_POS          = (SCREEN_WIDTH // 2,  50)
    PID_TITLE_POS      = (170              , 290)
    STATE_FEEDBACK_POS = (170              , 490)
    LQR_TITLE_POS      = (170              , 690)
    MPC_TITLE_POS      = (170              , 890)

    # Colors
    SCREEN_COLOR = (  0,   0,   0)  # Black
    TITLE_COLOR  = (255, 153,  52)  # Orange
    TEXT_COLOR   = (255, 178, 102)  # Light Orange

    # Font sizes
    TITLE_SIZE    = 80
    SUBTITLE_SIZE = 50

    # Canvas
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # Displays
    pygame.display.set_caption("Cart Pole Control")
    titleDisplay              = screen_tools.Text(screen, TITLE_POS         , TITLE_SIZE   , TITLE_COLOR)
    pidTitleDisplay           = screen_tools.Text(screen, PID_TITLE_POS     , SUBTITLE_SIZE, TEXT_COLOR )
    stateFeedBackTitleDisplay = screen_tools.Text(screen, STATE_FEEDBACK_POS, SUBTITLE_SIZE, TEXT_COLOR )
    lqrTitleDisplay           = screen_tools.Text(screen, LQR_TITLE_POS     , SUBTITLE_SIZE, TEXT_COLOR )
    mpcTitleDisplay           = screen_tools.Text(screen, MPC_TITLE_POS     , SUBTITLE_SIZE, TEXT_COLOR )


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(SCREEN_COLOR)

        # Draw titles
        titleDisplay.draw("Cart Pole - Control")
        pidTitleDisplay.draw("PID")
        stateFeedBackTitleDisplay.draw("State Feedback")
        lqrTitleDisplay.draw("LQR")
        mpcTitleDisplay.draw("MPC")

        #pygame.draw.line(screen, (255, 255, 255), (320, 0), (320, SCREEN_HEIGHT), 2)  # Vertical line
        #pygame.draw.line(screen, (255, 255, 255), (0, 100), (SCREEN_WIDTH, 100), 2)  # Horizontal line

        # Draw rectangles for each control method
        height = 180
        pygame.draw.rect(screen, (255, 255, 255), (340, 200, int(height * (1 + np.sqrt(2))), height), 2)   # PID rectangle
        pygame.draw.rect(screen, (255, 255, 255), (840, 200, int(height * (1 + 2*np.sqrt(2))), height), 2)   # PID rectangle 2

        pygame.draw.rect(screen, (255, 255, 255), (340, 400, int(height * (1 + np.sqrt(2))), height), 2)   # State Feedback rectangle
        pygame.draw.rect(screen, (255, 255, 255), (840, 400, int(height * (1 + 2*np.sqrt(2))), height), 2)   # State Feedback rectangle 2

        pygame.draw.rect(screen, (255, 255, 255), (340, 600, int(height * (1 + np.sqrt(2))), height), 2)   # LQR rectangle
        pygame.draw.rect(screen, (255, 255, 255), (840, 600, int(height * (1 + 2*np.sqrt(2))), height), 2)   # LQR rectangle 2

        pygame.draw.rect(screen, (255, 255, 255), (340, 800, int(height * (1 + np.sqrt(2))), height), 2)   # MPC rectangle
        pygame.draw.rect(screen, (255, 255, 255), (840, 800, int(height * (1 + 2*np.sqrt(2))), height), 2)   # MPC rectangle 2



        pygame.display.flip()    # Update the display

    pygame.quit()
