import pygame
import numpy as np
from cart_pole import MAX_CART_DISPLACEMENT, INITIAL_CART_X, CartPole

print("Screen Tools Imported")

colors = {
    "black":        (  0,   0,   0),
    "white":        (255, 255, 255),
    "orange":       (255, 153,  52),
    "light_orange": (255, 178, 102),
    "gray":         (160, 160, 160)
}

# Environment sizes
SCREEN_HEIGHT = 1000
SCREEN_WIDTH  = 1690

# Font sizes
TITLE_SIZE    = 80
SUBTITLE_SIZE = 50

# Display Sizes
DISPLAY_HEIGHT = 180
DISPLAY_LENGTH = int(DISPLAY_HEIGHT * (1 + np.sqrt(2)))

# Positions
TITLE_POS                 = (SCREEN_WIDTH // 2,  50)
PID_TITLE_POS             = (170              , 290)
PID_CANVAS_POS            = (340              , 200)
PID_FORCE_POS             = (790              , 200)
PID_ERRORS_POS            = (1240             , 200)
STATE_FEEDBACK_POS        = (170              , 490)
LQR_TITLE_POS             = (170              , 690)
MPC_TITLE_POS             = (170              , 890)
STATE_FEEDBACK_CANVAS_POS = (340              , 400)
LQR_CANVAS_POS            = (340              , 600)
MPC_CANVAS_POS            = (340              , 800)

# Slider Sizes and positions
SLIDER_LENGTH    = DISPLAY_LENGTH
SLIDER_HEIGHT    = 20
SLIDER_POS       = (340, 150)
SLIDER_TITLE_POS = (170, 170)

# Meter stick
CART_DISP_RESOLUTION  = DISPLAY_LENGTH // (2 * MAX_CART_DISPLACEMENT)

class Canvas:
    """
    A class for drawing a canvas on a Pygame screen, including a meter stick for visual reference.
    Attributes:
        __screen (pygame.display): The Pygame display surface to draw on.
        __position (tuple): A tuple (x, y) representing the top-left corner of the canvas.
        __halfSteps (int): The number of half steps for the meter stick graduations.
        __meterStickPosY (int): The vertical position of the meter stick on the canvas.
        __resolution (int): The pixel resolution for each unit on the meter stick.
        __cartPole (CartPole): An instance of the CartPole class.
        
    Methods:
        __rel2abs_position(relPosition): Converts a relative position to an absolute position on the screen.
        __draw_meter_stick(): Draws a meter stick on the canvas, with a horizontal line and vertical graduation lines.
        draw_cart(): Draws the cart on the screen.
    """
    def __init__(self, screen:pygame.display, position:tuple, cartPole:CartPole):
        """
        Initializes the Canvas class.
        Args:
            screen (pygame.display): The Pygame display surface to draw on.
            position (tuple): A tuple (x, y) representing the top-left corner of the canvas.
            cartPole (CartPole): An instance of the CartPole class
        """
        self.__screen                      = screen
        self.__cartDisplaypos              = position
        self.__angleErrorDisplayPos        = (position[0] + 450, position[1])
        self.__displacementErrorDisplayPos = (position[0] + 900, position[1])
        self.__meterStickPosY              = 55
        self.__cartPole                    = cartPole

    def __rel2abs_position(self, relPosition:tuple, upLeftCoerner:tuple=(0, 0)):
        """
        Converts a relative position to an absolute position on the screen.
        Args:
            relPosition (tuple): A tuple (x, y) representing the relative position.
        Returns:
            tuple: A tuple (x, y) representing the absolute position.
        """
        xAbs = upLeftCoerner[0] + relPosition[0]
        yAbs = upLeftCoerner[1] + (DISPLAY_HEIGHT - relPosition[1])

        return (xAbs, yAbs)
    
    def __draw_meter_stick(self):
        """
        Draws a meter stick on the canvas, with a horizontal line and vertical graduation lines.
        """
        meterStickPosStart = self.__rel2abs_position((0, self.__meterStickPosY), self.__cartDisplaypos)
        meterStickPosEnd   = self.__rel2abs_position((DISPLAY_LENGTH, self.__meterStickPosY), self.__cartDisplaypos)

        # Horizontal line
        pygame.draw.line(self.__screen, colors["black"], meterStickPosStart, meterStickPosEnd, width=2)

        # Vertical lines
        originX = DISPLAY_LENGTH // 2
        for i in range(int(MAX_CART_DISPLACEMENT) + 1):
            vertLineLen    = 20
            vertLineX      = originX + (i * CART_DISP_RESOLUTION)
            vertLineXNeg   = originX - (i * CART_DISP_RESOLUTION)
            vertLineYStart = self.__meterStickPosY - vertLineLen // 2
            vertLineYEnd   = vertLineYStart + vertLineLen

            vertLineStart = self.__rel2abs_position((vertLineX, vertLineYStart), self.__cartDisplaypos)
            vertLineEnd   = self.__rel2abs_position((vertLineX, vertLineYEnd), self.__cartDisplaypos)

            vertLineStartNeg = self.__rel2abs_position((vertLineXNeg, vertLineYStart), self.__cartDisplaypos)
            vertLineEndNeg   = self.__rel2abs_position((vertLineXNeg, vertLineYEnd), self.__cartDisplaypos)

            pygame.draw.line(self.__screen, colors["black"], vertLineStart   , vertLineEnd   , width=2)
            if i > 0:
                pygame.draw.line(self.__screen, colors["black"], vertLineStartNeg, vertLineEndNeg, width=2)
            
            # Scale graduation
            scaleTextPos = self.__rel2abs_position((vertLineX, vertLineYStart - 10), self.__cartDisplaypos)
            scaleText    = Text(self.__screen, scaleTextPos, 20, colors["black"])
            scaleText.draw(str(i))

            if i > 0:
                scaleTextPosNeg = self.__rel2abs_position((vertLineXNeg, vertLineYStart - 10), self.__cartDisplaypos)
                scaleTextNeg    = Text(self.__screen, scaleTextPosNeg, 20, colors["black"])
                scaleTextNeg.draw(str(-i))

    def draw_cart(self):
        """
        Draws the cart on the screen. The cart's position and pole angle are determined by the attributes of the CartPole instance.
        The cart is represented as a rectangle, the wheels as circles, and the pole as a line with circles at the ends. The meter stick is also drawn for reference.
        """
        scale          = 80  # pixels per meter
        cartX          = int(self.__cartPole.cartX * CART_DISP_RESOLUTION + DISPLAY_LENGTH // 2)
        cartLength     = int(scale * self.__cartPole.length)
        halfCartLength = cartLength // 2
        CartHeight     = int(scale * self.__cartPole.height)
        cartWheelR     = int(scale * self.__cartPole.wheelRadius)
        wheelBase      = cartLength // 2
        poleLength     = int(scale * self.__cartPole.poleLength)
        poleAngle      = self.__cartPole.poleAngle

        # Draw cart body
        cartTopLeftCorner    = (cartX - halfCartLength, self.__meterStickPosY + cartWheelR + CartHeight)
        cartTopLeftCornerAbs = self.__rel2abs_position(cartTopLeftCorner, self.__cartDisplaypos)

        pygame.draw.rect(self.__screen, colors["orange"], (cartTopLeftCornerAbs[0], cartTopLeftCornerAbs[1], cartLength, CartHeight))
        pygame.draw.rect(self.__screen, colors["black"] , (cartTopLeftCornerAbs[0], cartTopLeftCornerAbs[1], cartLength, CartHeight), width=1)

        # Draw wheels
        leftWheelPos     = (cartX - wheelBase // 2, self.__meterStickPosY + cartWheelR)
        leftWheelPosAbs  = self.__rel2abs_position(leftWheelPos, self.__cartDisplaypos)
        rightWheelPos    = (cartX + wheelBase // 2, self.__meterStickPosY + cartWheelR)
        rightWheelPosAbs = self.__rel2abs_position(rightWheelPos, self.__cartDisplaypos)

        pygame.draw.circle(self.__screen, colors["light_orange"], leftWheelPosAbs , cartWheelR)
        pygame.draw.circle(self.__screen, colors["light_orange"], rightWheelPosAbs, cartWheelR)
        pygame.draw.circle(self.__screen, colors["black"]       , leftWheelPosAbs , cartWheelR, width=1)
        pygame.draw.circle(self.__screen, colors["black"]       , rightWheelPosAbs, cartWheelR, width=1)

        # Draw pole
        poleStart = (cartX, self.__meterStickPosY + cartWheelR + CartHeight//2)
        poleEnd   = (int(poleStart[0] + poleLength * np.sin(poleAngle)), int(poleStart[1] + poleLength * np.cos(poleAngle)))

        poleStartAbs = self.__rel2abs_position(poleStart, self.__cartDisplaypos)
        poleEndAbs   = self.__rel2abs_position(poleEnd  , self.__cartDisplaypos)

        pygame.draw.line(self.__screen, colors["gray"], poleStartAbs, poleEndAbs, width=10)
        pygame.draw.circle(self.__screen, colors["gray"], poleStartAbs, 4)
        pygame.draw.circle(self.__screen, colors["gray"], poleEndAbs, 4)

        # Draw meter stick
        self.__draw_meter_stick()

    def plot_angle_error(self, angleErrorHistory):

        # Plot the angle error history
        pass

        
        

class Text:
    """
    A class for rendering text on a Pygame screen.
    Attributes:
        __screen (pygame.display): The Pygame display surface to draw on.
        __position (tuple): A tuple (x, y) representing the position of the text's center.
        __color (tuple): A tuple (R, G, B) representing the color of the text.
        __font (pygame.font.Font): The Pygame font object used to render the text.

    Methods:
        draw(message): Draws the specified message on the screen at the initialized position, size, and color.
    """
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


class SlideBar:
    """
    A class for creating a horizontal slider bar on a Pygame screen, allowing the user to select a value by dragging a selector along the bar.
    Attributes:
        __setPoint (float): The current value selected by the slider, calculated based on the position of the selector.
        __screen (pygame.display): The Pygame display surface to draw on.
        __sliderPos (int): The current horizontal position of the slider's selector.
        __rect (pygame.Rect): A Pygame Rect object representing the area of the slider bar.
    Methods:
        draw(): Draws the slider bar and selector on the screen.
        handle_event(event): Handles mouse events for dragging the selector.
    """
    def __init__(self, screen):
        """
        Initializes the SlideBar class.
        Args:            screen (pygame.display): The Pygame display surface to draw on.
        """
        self.__setPoint  = INITIAL_CART_X
        self.__screen    = screen
        self.__sliderPos = SLIDER_POS[0] + INITIAL_CART_X * CART_DISP_RESOLUTION + DISPLAY_LENGTH // 2
        self.__rect      = pygame.Rect(SLIDER_POS[0], SLIDER_POS[1], SLIDER_LENGTH, SLIDER_HEIGHT)

    def draw(self):
        """
        Draws the slider bar and selector on the screen.
        """
        pygame.draw.rect(self.__screen, colors["gray"], self.__rect)

        # Selector position
        pos = np.array((self.__sliderPos, SLIDER_POS[1] + SLIDER_HEIGHT//2)).astype(int)
        pygame.draw.circle(self.__screen, colors["light_orange"], pos, SLIDER_HEIGHT//2 + 2)

    def get_set_point(self):
        """
        Returns the current value selected by the slider.
        Returns:
            float: The current set point value based on the position of the selector.
        """
        return self.__setPoint

    def handle_event(self, event):
        """
        Handles mouse events for dragging the selector.
        Args:
            event (pygame.event.Event): The Pygame event to handle, expected to be a MOUSEMOTION event.
        """
        if self.__rect.collidepoint(event.pos):
            self.__sliderPos = event.pos[0]

            relPos = event.pos[0] - SLIDER_POS[0]
            self.__setPoint = relPos / (CART_DISP_RESOLUTION) - MAX_CART_DISPLACEMENT



def draw_static_screen(screen:pygame.display):
    # Draw background, except for the rectangles where the control methods will be displayed
    pygame.draw.rect(screen, colors["black"], (                   0, 0,          340, SCREEN_HEIGHT))  # Fill the background with black
    pygame.draw.rect(screen, colors["black"], (340                 , 0, SCREEN_WIDTH,           200))  # Fill the top area with black
    pygame.draw.rect(screen, colors["black"], (340 + DISPLAY_LENGTH, 0, SCREEN_WIDTH, SCREEN_HEIGHT))  # Fill the left area with black
    
    # Fill the areas below the rectangles with black
    pygame.draw.rect(screen, colors["black"], (PID_CANVAS_POS[0]           , PID_CANVAS_POS[1]            + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below PID
    pygame.draw.rect(screen, colors["black"], (STATE_FEEDBACK_CANVAS_POS[0], STATE_FEEDBACK_CANVAS_POS[1] + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below State Feedback
    pygame.draw.rect(screen, colors["black"], (LQR_CANVAS_POS[0]           , LQR_CANVAS_POS[1]            + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below LQR
    pygame.draw.rect(screen, colors["black"], (MPC_CANVAS_POS[0]           , MPC_CANVAS_POS[1]            + DISPLAY_HEIGHT, DISPLAY_LENGTH, 20))  # Below MPC

    # Displays
    titleDisplay              = Text(screen, TITLE_POS         , TITLE_SIZE   , colors["orange"])
    setPointText              = Text(screen, SLIDER_TITLE_POS  , SUBTITLE_SIZE, colors["orange"])
    pidTitleDisplay           = Text(screen, PID_TITLE_POS     , SUBTITLE_SIZE, colors["orange"])
    stateFeedBackTitleDisplay = Text(screen, STATE_FEEDBACK_POS, SUBTITLE_SIZE, colors["orange"])
    lqrTitleDisplay           = Text(screen, LQR_TITLE_POS     , SUBTITLE_SIZE, colors["orange"])
    mpcTitleDisplay           = Text(screen, MPC_TITLE_POS     , SUBTITLE_SIZE, colors["orange"])

    # Draw titles
    titleDisplay.draw("Cart Pole - Control")
    pidTitleDisplay.draw("PID")
    stateFeedBackTitleDisplay.draw("State Feedback")
    lqrTitleDisplay.draw("LQR")
    mpcTitleDisplay.draw("MPC")
    setPointText.draw("Set Point")

    #pygame.draw.line(screen, (255, 255, 255), (320, 0), (320, SCREEN_HEIGHT), 2)  # Vertical line
    #pygame.draw.line(screen, (255, 255, 255), (0, 100), (SCREEN_WIDTH, 100), 2)  # Horizontal line

    # Draw rectangles for each control method
    #height = 180
    #pygame.draw.rect(screen, (255, 255, 255), (340, 200, int(height * (1 + np.sqrt(2))), height), 2)   # PID rectangle
    pygame.draw.rect(screen, (255, 255, 255), (PID_FORCE_POS[0], PID_FORCE_POS[1], DISPLAY_LENGTH, DISPLAY_HEIGHT))   # PID rectangle 2
    pygame.draw.rect(screen, (255, 255, 255), (PID_ERRORS_POS[0], PID_ERRORS_POS[1], DISPLAY_LENGTH, DISPLAY_HEIGHT))   # PID rectangle 3

    #pygame.draw.rect(screen, (255, 255, 255), (340, 400, int(height * (1 + np.sqrt(2))), height), 2)   # State Feedback rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 400, int(height * (1 + 2*np.sqrt(2))), height), 2)   # State Feedback rectangle 2

    #pygame.draw.rect(screen, (255, 255, 255), (340, 600, int(height * (1 + np.sqrt(2))), height), 2)   # LQR rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 600, int(height * (1 + 2*np.sqrt(2))), height), 2)   # LQR rectangle 2

    #pygame.draw.rect(screen, (255, 255, 255), (340, 800, int(height * (1 + np.sqrt(2))), height), 2)   # MPC rectangle
    #pygame.draw.rect(screen, (255, 255, 255), (840, 800, int(height * (1 + 2*np.sqrt(2))), height), 2)   # MPC rectangle 2
