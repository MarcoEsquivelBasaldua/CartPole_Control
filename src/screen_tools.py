import pygame

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
