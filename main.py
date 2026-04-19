import pygame
import sys
import numpy as np
sys.path.insert(1, './src')
import cart_pole
import controllers
import screen_tools

if __name__ == "__main__":
    pygame.init()

    # Canvas
    screen = pygame.display.set_mode((screen_tools.SCREEN_WIDTH, screen_tools.SCREEN_HEIGHT))

    # Displays
    pygame.display.set_caption("Cart Pole Control")


    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(screen_tools.colors["white"])  # Clear the screen with white background

        screen_tools.draw_static_screen(screen)

        



        pygame.display.flip()    # Update the display

    pygame.quit()
