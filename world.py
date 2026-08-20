import math
import pygame
import heapq

from robot import Robot
from controller import Angle_Controller
from obstacle import Obstacle
import settings

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
                            settings.START_CIRCLE_COLOR,
                            self.start_pos,
                            settings.JOINT_RADIUS,
                            width = 0
                        )

        # draws the green end icon
        if self.end_pos:
            pygame.draw.circle(
                                    screen,
                                    settings.END_CIRCLE_COLOR,
                                    self.end_pos,
                                    settings.JOINT_RADIUS,
                                    width = 0
                                )

#--- World ------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# owns everything on the screen
class World:
    def __init__(self):

        self.robot = Robot()
        self.angle_controller = Angle_Controller(
            settings.CONTROL_BACKGROUND_X,
            settings.CONTROL_BACKGROUND_Y, 
            settings.BACKGROUND_WIDTH, 
            settings.BACKGROUND_HEIGHT)

        self.path_controller = Angle_Controller(
                    settings.CONTROL_BACKGROUND_X,
                    settings.CONTROL_BACKGROUND_Y + 100, 
                    settings.BACKGROUND_WIDTH * 0.75, 
                    settings.BACKGROUND_HEIGHT)

        self.start_angle_display = Angle_Controller(
                            settings.CONTROL_BACKGROUND_X,
                            settings.CONTROL_BACKGROUND_Y + 200, 
                            settings.BACKGROUND_WIDTH * 0.75, 
                            settings.BACKGROUND_HEIGHT)

        self.end_angle_display = Angle_Controller(
                                    settings.CONTROL_BACKGROUND_X,
                                    settings.CONTROL_BACKGROUND_Y + 300, 
                                    settings.BACKGROUND_WIDTH * 0.75, 
                                    settings.BACKGROUND_HEIGHT)
        
        self.angle_controller.panel_maker(self.robot, settings.ANGLE_DISPLAY_PANEL_NUMBER)
        self.angle_controller.set_labels(self.robot, settings.ANGLE_DISPLAY_PANEL_NUMBER)

        self.path_controller.function_panel_maker(self.robot, 2, 1)
        self.path_controller.update_panels(2)

        self.start_angle_display.panel_maker(self.robot, 2, 2)
        self.end_angle_display.panel_maker(self.robot, 2, 3)

        self.icons = Path_Icons()

        self.obstacles = []
        for i in range(settings.OBSTACLE_NUMBER):
            self.obstacles.append(Obstacle(i))