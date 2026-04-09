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
    TITLE_POS = (SCREEN_WIDTH // 2, 50)

    # Colors
    SCREEN_COLOR = (  0,   0,   0)  # Black
    TITLE_COLOR  = (255, 153,  52)  # Orange
    TEXT_COLOR   = (255, 178, 102)  # Light Orange

    # Font sizes
    TITLE_SIZE = 80

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # Displays
    pygame.display.set_caption("Cart Pole Control")
    titleDisplay = screen_tools.Text(screen, TITLE_POS, TITLE_SIZE, TITLE_COLOR)


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(SCREEN_COLOR)

        titleDisplay.draw("Cart Pole - Control")

        pygame.display.flip()    # Update the display

    pygame.quit()
