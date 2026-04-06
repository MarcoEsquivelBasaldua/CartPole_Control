import pygame
import sys
sys.path.insert(1, './src')
import cart_pole
import controllers
import screen_tools

if __name__ == "__main__":
    pygame.init()

    # Environment sizes
    ENV_HEIGHT    = 900
    ENV_WIDTH     = 1400

    # Colors
    SCREEN_COLOR = (  0,   0,   0)  # Black
    TITLE_COLOR  = (255, 153,  52)  # Orange
    TEXT_COLOR   = (255, 178, 102)  # Light Orange

    screen = pygame.display.set_mode((ENV_WIDTH, ENV_HEIGHT))
    pygame.display.set_caption("Cart Pole Control")


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(SCREEN_COLOR)

        pygame.display.flip()    # Update the display

    pygame.quit()
