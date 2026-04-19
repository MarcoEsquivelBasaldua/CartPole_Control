import pygame
import numpy as np

colors = {
    "black":        (  0,   0,   0),
    "white":        (255, 255, 255),
    "orange":       (255, 153,  52),
    "light_orange": (255, 178, 102)
}

# Environment sizes
SCREEN_HEIGHT = 1000
SCREEN_WIDTH  = 1620

# Positions
TITLE_POS          = (SCREEN_WIDTH // 2,  50)
PID_TITLE_POS      = (170              , 290)
STATE_FEEDBACK_POS = (170              , 490)
LQR_TITLE_POS      = (170              , 690)
MPC_TITLE_POS      = (170              , 890)

# Font sizes
TITLE_SIZE    = 80
SUBTITLE_SIZE = 50

# Display Sizes
DISPLAY_HEIGHT = 180
DISPLAY_LENGTH = int(DISPLAY_HEIGHT * (1 + np.sqrt(2)))

class Text:
    def __init__(self, screen:pygame.display, position:tuple, size:int, color:tuple):
        """
        Initializes the Text class.
        Args:
            screen (pygame.display): The Pygame display surface to draw on.
            position (tuple): A tuple (x, y) representing the position of the text's center.
            size (int): The font size of the text.
            color (tuple): A tuple (R, G, B) representing the color of the text.
        """
        self.__screen   = screen
        self.__position = position
        self.__color    = color
        self.__font     = pygame.font.Font(None, size)

    def draw(self, message:str):
        """
        Draws the specified message on the screen at the initialized position, size, and color.
        Args:
            message (str): The text message to be displayed.
        """
        textSurface = self.__font.render(message, True, self.__color)
        textRect    = textSurface.get_rect(center = self.__position)

        self.__screen.blit(textSurface, textRect)


def draw_static_screen(screen:pygame.display):
    # Draw background, except for the rectangles where the control methods will be displayed
    pygame.draw.rect(screen, colors["black"], (                   0,            0,          340, SCREEN_HEIGHT))  # Fill the background with black
    pygame.draw.rect(screen, colors["black"], (340                 ,            0, SCREEN_WIDTH,           200))  # Fill the top area with black
    pygame.draw.rect(screen, colors["black"], (340 + DISPLAY_LENGTH,            0, SCREEN_WIDTH, SCREEN_HEIGHT))  # Fill the left area with black
    
    # Fill the areas below the rectangles with black
    pygame.draw.rect(screen, colors["black"], (340, 200 + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below PID
    pygame.draw.rect(screen, colors["black"], (340, 400 + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below State Feedback
    pygame.draw.rect(screen, colors["black"], (340, 600 + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below LQR
    pygame.draw.rect(screen, colors["black"], (340, 800 + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below MPC

    # Displays
    titleDisplay              = Text(screen, TITLE_POS         , TITLE_SIZE   , colors["orange"])
    pidTitleDisplay           = Text(screen, PID_TITLE_POS     , SUBTITLE_SIZE, colors["orange"] )
    stateFeedBackTitleDisplay = Text(screen, STATE_FEEDBACK_POS, SUBTITLE_SIZE, colors["orange"] )
    lqrTitleDisplay           = Text(screen, LQR_TITLE_POS     , SUBTITLE_SIZE, colors["orange"] )
    mpcTitleDisplay           = Text(screen, MPC_TITLE_POS     , SUBTITLE_SIZE, colors["orange"] )

    # Draw titles
    titleDisplay.draw("Cart Pole - Control")
    pidTitleDisplay.draw("PID")
    stateFeedBackTitleDisplay.draw("State Feedback")
    lqrTitleDisplay.draw("LQR")
    mpcTitleDisplay.draw("MPC")

    #pygame.draw.line(screen, (255, 255, 255), (320, 0), (320, SCREEN_HEIGHT), 2)  # Vertical line
    #pygame.draw.line(screen, (255, 255, 255), (0, 100), (SCREEN_WIDTH, 100), 2)  # Horizontal line

    # Draw rectangles for each control method
    #height = 180
    #pygame.draw.rect(screen, (255, 255, 255), (340, 200, int(height * (1 + np.sqrt(2))), height), 2)   # PID rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 200, int(height * (1 + 2*np.sqrt(2))), height), 2)   # PID rectangle 2

    #pygame.draw.rect(screen, (255, 255, 255), (340, 400, int(height * (1 + np.sqrt(2))), height), 2)   # State Feedback rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 400, int(height * (1 + 2*np.sqrt(2))), height), 2)   # State Feedback rectangle 2

    #pygame.draw.rect(screen, (255, 255, 255), (340, 600, int(height * (1 + np.sqrt(2))), height), 2)   # LQR rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 600, int(height * (1 + 2*np.sqrt(2))), height), 2)   # LQR rectangle 2

    #pygame.draw.rect(screen, (255, 255, 255), (340, 800, int(height * (1 + np.sqrt(2))), height), 2)   # MPC rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 800, int(height * (1 + 2*np.sqrt(2))), height), 2)   # MPC rectangle 2
