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
                      
#--- Controller --------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Angle_Controller():
    def __init__(self, robot):

        # creates the background rectangle for the controller
        self.angle_control_background = pygame.Rect(
            CONTROL_BACKGROUND_X,
            CONTROL_BACKGROUND_Y, 
            BACKGROUND_WIDTH, 
            BACKGROUND_HEIGHT
        )

        self.font = pygame.font.Font(None, 35)

        # this variable determines the rate of change of the joint angle
        self.angle_speed = 1

        #these lists hold the panels and the angle labels within the crontoller  
        self.inner_panels = []
        self.angle_labels = []

        # adds the first panel meant for displaying angle speed
        self.inner_panels.append(pygame.Rect(
            INNER_PANEL_X,
            INNER_PANEL_Y,
            INNER_PANEL_DIMENSIONS,
            INNER_PANEL_DIMENSIONS
        ))

        # adds the remaining panels alongside the respective angles they need to display
        for i in range(ANGLE_DISPLAY_PANEL_NUMBER):

            self.inner_panels.append(pygame.Rect(
                INNER_PANEL_X,
                INNER_PANEL_Y + 70*(i+1),
                INNER_PANEL_DIMENSIONS,
                INNER_PANEL_DIMENSIONS
            ))

            # displays current joint angle
            angle = int(math.degrees(robot.relative_angles[i]))
            self.angle_labels.append(self.font.render(f"{angle}", True, TEXT_COLOR))

    def update_labels(self, robot):
        for i in range(len(self.inner_panels)):
            if i == 0:
                self.speed_label = self.font.render(f"{self.angle_speed}", True, TEXT_COLOR)
            else:
                angle = int(math.degrees(robot.relative_angles[i-1]))
                self.angle_labels[i-1] = (self.font.render(f"{angle}", True, TEXT_COLOR))

    # draws the angle controller
    def draw_controller(self, screen, robot):

        # draws the background panel for the controller
        pygame.draw.rect(
            screen, 
            CONTROL_BACKGROUND_COLOR,
            self.angle_control_background
        )

        # draws the inner panels alongside their labels
        for i in range(len(self.inner_panels)):

            # draws each inner panel
            pygame.draw.rect(screen, INNER_PANEL_COLOR, self.inner_panels[i])

            #labels the angle speed
            if i == 0:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.speed_label, (center_x-12, center_y-12))

            #labels the joint angles for the remaining panels
            else:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.angle_labels[i-1], (center_x-12, center_y-12))