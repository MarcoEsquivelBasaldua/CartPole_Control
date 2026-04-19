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

    # Canvas
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))

    # Displays
    pygame.display.set_caption("Cart Pole Control")


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen_tools.draw_static_screen(screen)

        



        pygame.display.flip()    # Update the display

    pygame.quit()
