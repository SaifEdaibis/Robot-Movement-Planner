import math
import pygame
import heapq

import settings
                      
#--- Controller --------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Angle_Controller():
    def __init__(self, robot):

        # creates the background rectangle for the controller
        self.angle_control_background = pygame.Rect(
            settings.CONTROL_BACKGROUND_X,
            settings.CONTROL_BACKGROUND_Y, 
            settings.BACKGROUND_WIDTH, 
            settings.BACKGROUND_HEIGHT
        )

        self.font = pygame.font.Font(None, 35)

        # this variable determines the rate of change of the joint angle
        self.angle_speed = 1

        #these lists hold the panels and the angle labels within the crontoller  
        self.inner_panels = []
        self.angle_labels = []

        # adds the first panel meant for displaying angle speed
        self.inner_panels.append(pygame.Rect(
            settings.INNER_PANEL_X,
            settings.INNER_PANEL_Y,
            settings.INNER_PANEL_DIMENSIONS,
            settings.INNER_PANEL_DIMENSIONS
        ))

        # adds the remaining panels alongside the respective angles they need to display
        for i in range(settings.ANGLE_DISPLAY_PANEL_NUMBER):

            self.inner_panels.append(pygame.Rect(
                settings.INNER_PANEL_X,
                settings.INNER_PANEL_Y + 70*(i+1),
                settings.INNER_PANEL_DIMENSIONS,
                settings.INNER_PANEL_DIMENSIONS
            ))

            # displays current joint angle
            angle = int(math.degrees(robot.relative_angles[i]))
            self.angle_labels.append(self.font.render(f"{angle}", True, settings.TEXT_COLOR))

    def update_labels(self, robot):
        for i in range(len(self.inner_panels)):
            if i == 0:
                self.speed_label = self.font.render(f"{self.angle_speed}", True, settings.TEXT_COLOR)
            else:
                angle = int(math.degrees(robot.relative_angles[i-1]))
                self.angle_labels[i-1] = (self.font.render(f"{angle}", True, settings.TEXT_COLOR))

    # draws the angle controller
    def draw_controller(self, screen, robot):

        # draws the background panel for the controller
        pygame.draw.rect(
            screen, 
            settings.CONTROL_BACKGROUND_COLOR,
            self.angle_control_background
        )

        # draws the inner panels alongside their labels
        for i in range(len(self.inner_panels)):

            # draws each inner panel
            pygame.draw.rect(screen, settings.INNER_PANEL_COLOR, self.inner_panels[i])

            #labels the angle speed
            if i == 0:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.speed_label, (center_x-12, center_y-12))

            #labels the joint angles for the remaining panels
            else:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.angle_labels[i-1], (center_x-12, center_y-12))