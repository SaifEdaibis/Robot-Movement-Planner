import math
import pygame
import heapq

import settings

#--- Main Screen ------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Owns the screen and display and all associated functions
class Front_Display:
    def __init__(self):

        # Sets up the main screen
        self.screen = pygame.display.set_mode((settings.SCREEN_WIDTH, settings.SCREEN_HEIGHT))
        pygame.display.set_caption('Robotic Arm Movement Planner')

        # Sets up the screen timing
        self.clock = pygame.time.Clock()


    # makes the background grid for the screen
    def draw_grid(self):

        # makes the rows for the grid
        for line in range(int((settings.SCREEN_HEIGHT / settings.GRID_DIMENSION)-1)):

            start_pos = (
                0,
                settings.GRID_DIMENSION * (line +1)
            )

            end_pos = (
                settings.SCREEN_WIDTH, 
                settings.GRID_DIMENSION * (line +1)
            )

            pygame.draw.line(
                self.screen,
                settings.GRID_COLOR,
                start_pos,
                end_pos,
                settings.GRID_LINE_WIDTH
            )

        # makes the coloumns for the grid
        for line in range(int((settings.SCREEN_WIDTH / settings.GRID_DIMENSION)-1)):

            start_pos = (
                settings.GRID_DIMENSION * (line +1),
                0
            )
            end_pos = (
                settings.GRID_DIMENSION * (line +1),
                settings.SCREEN_HEIGHT
            )

            pygame.draw.line(
                self.screen,
                settings.GRID_COLOR,
                start_pos,
                end_pos,
                settings.GRID_LINE_WIDTH
            )

    #draws everything owned by the world
    def draw_world(self, world):
        
        world.icons.draw_icons(self.screen)

        world.robot.draw_robot(self.screen)

        world.angle_controller.update_labels(world.robot)
        world.angle_controller.draw_controller(self.screen, world.robot)

        for i in range(settings.OBSTACLE_NUMBER):
            world.obstacles[i].draw_obstacle(self.screen)
