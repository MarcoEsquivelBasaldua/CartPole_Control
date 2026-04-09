import pygame
import sys
sys.path.insert(1, './src')
import cart_pole
import controllers
import screen_tools

if __name__ == "__main__":
    pygame.init()

    # Environment sizes
    SCREEN_HEIGHT = 900
    SCREEN_WIDTH  = 1400

    # Positions
    TITLE_POS          = (SCREEN_WIDTH // 2,  50)
    PID_TITLE_POS      = (180              , 200)
    STATE_FEEDBACK_POS = (180              , 300)
    LQR_TITLE_POS      = (180              , 500)
    MPC_TITLE_POS      = (180              , 700)

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

        pygame.display.flip()    # Update the display

    pygame.quit()
