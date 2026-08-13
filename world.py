import math
import pygame
import heapq

from robot import Robot
from controller import Angle_Controller
from obstacle import Obstacle


# Screen Info
SCREEN_WIDTH = 600              ### pixels
SCREEN_HEIGHT = 600
BACKGROUND_COLOR = (255, 255, 255)

# Timing Info
FRAMERATE = 60

# Grid Info
GRID_DIMENSION = 40             ### pixels
GRID_COLOR = (0, 0, 0)
GRID_LINE_WIDTH = 2             ### pixels

# Robot Info
BASE_JOINT = (300, 350)         ### pixels

BASE_JOINT_ANGLE = math.pi/2
SECOND_JOINT_ANGLE = math.pi/2
THIRD_JOINT_ANGLE = math.pi/2

STARTING_ARM_LENGTH_ONE = 100
STARTING_ARM_LENGTH_TWO = 100
STARTING_ARM_LENGTH_THREE = 100

JOINT_NUMBERS = 4
ARM_COLOR = (0, 0, 0)
JOINT_COLOR = (212, 175, 55)
JOINT_RADIUS = 9
INNER_JOINT_RADIUS = 5
ARM_WIDTH = 7                  ### pixels

# Angle Controller
CONTROL_BACKGROUND_X = 20
CONTROL_BACKGROUND_Y = 40
BACKGROUND_WIDTH = 90
BACKGROUND_HEIGHT = 300
CONTROL_BACKGROUND_COLOR = (128,128,128)

ANGLE_DISPLAY_PANEL_NUMBER = 3
INNER_PANEL_X = 40
INNER_PANEL_Y = 60
INNER_PANEL_DIMENSIONS = 50
INNER_PANEL_COLOR = (255,255,255)

TEXT_COLOR = (0,0,0)

# Icons Info
START_CIRCLE_COLOR = (128, 0, 0)
END_CIRCLE_COLOR = (0, 100, 0)

# Obstacle Info
OBSTACLE_COLOR = (135, 206, 235)
OBSTACLE_START_X = 180
OBSTACLE_START_Y = 400
OBSTACLE_DIMENSIONS = 50

# Route Info
RESOLUTION = 5
COLLISION_PADDING = ARM_WIDTH
JOINT_COLLISION_PADDING = JOINT_RADIUS * 2
WEIGHT = 1.5

#--- Path Icons ------------------------------------------------------------------------------------------------------------------------------------------

class Path_Icons:
    def __init__(self):

        # red start icon position
        self.start_pos = None

        # green end icon position
        self.end_pos = None

    def draw_icons(self, screen):

        # draws the red start icon
        if self.start_pos:
            pygame.draw.circle(
                            screen,
                            START_CIRCLE_COLOR,
                            self.start_pos,
                            JOINT_RADIUS,
                            width = 0
                        )

        # draws the green end icon
        if self.end_pos:
            pygame.draw.circle(
                                    screen,
                                    END_CIRCLE_COLOR,
                                    self.end_pos,
                                    JOINT_RADIUS,
                                    width = 0
                                )

#--- World ------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# owns everything on the screen
class World:
    def __init__(self):
        self.robot = Robot()
        self.angle_controller = Angle_Controller(self.robot)
        self.icons = Path_Icons()

        self.obstacles = []
        for i in range(3):
            self.obstacles.append(Obstacle(i))