import math
import pygame
import heapq

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

#--- Main Screen ------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Owns the screen and display and all associated functions
class Front_Display:
    def __init__(self):

        # Sets up the main screen
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption('Robotic Arm Movement Planner')

        # Sets up the screen timing
        self.clock = pygame.time.Clock()


    # makes the background grid for the screen
    def draw_grid(self):

        # makes the rows for the grid
        for line in range(int((SCREEN_HEIGHT / GRID_DIMENSION)-1)):

            start_pos = (
                0,
                GRID_DIMENSION * (line +1)
            )

            end_pos = (
                SCREEN_WIDTH, 
                GRID_DIMENSION * (line +1)
            )

            pygame.draw.line(
                self.screen,
                GRID_COLOR,
                start_pos,
                end_pos,
                GRID_LINE_WIDTH
            )

        # makes the coloumns for the grid
        for line in range(int((SCREEN_WIDTH / GRID_DIMENSION)-1)):

            start_pos = (
                GRID_DIMENSION * (line +1),
                0
            )
            end_pos = (
                GRID_DIMENSION * (line +1),
                SCREEN_HEIGHT
            )

            pygame.draw.line(
                self.screen,
                GRID_COLOR,
                start_pos,
                end_pos,
                GRID_LINE_WIDTH
            )

    #draws everything owned by the world
    def draw_world(self, world):
        
        world.icons.draw_icons(self.screen)

        world.robot.draw_robot(self.screen)

        world.angle_controller.update_labels(world.robot)
        world.angle_controller.draw_controller(self.screen, world.robot)

        for i in range(3):
            world.obstacles[i].draw_obstacle(self.screen)
