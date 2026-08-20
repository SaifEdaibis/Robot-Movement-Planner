import math
import pygame
import heapq

import settings
                      
#--- Controller --------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Angle_Controller():
    def __init__(self, x , y, width, height):

        # creates the background rectangle for the controller
        self.angle_control_background = pygame.Rect(
            x,
            y, 
            width, 
            height
        )

        self.font = pygame.font.Font(None, 35)

        # this variable determines the rate of change of the joint angle
        self.angle_speed = 1

    def panel_maker(self, robot, panel, row = 0):

        #these lists hold the panels and the angle labels within the crontoller  
        self.inner_panels = []
        self.angle_labels = []

        # adds the first panel meant for displaying angle speed
        self.inner_panels.append(pygame.Rect(
            settings.INNER_PANEL_X,
            settings.INNER_PANEL_Y + row * 100,
            settings.INNER_PANEL_DIMENSIONS,
            settings.INNER_PANEL_DIMENSIONS
        ))

        # adds the remaining panels alongside the respective angles they need to display
        for i in range(panel):

            self.inner_panels.append(pygame.Rect(
                settings.INNER_PANEL_X + 70*(i+1),
                settings.INNER_PANEL_Y + row * 100,
                settings.INNER_PANEL_DIMENSIONS,
                settings.INNER_PANEL_DIMENSIONS
            ))

    def set_labels(self, robot, panel):

        # adds the remaining angles they need to display
        for i in range(panel):
        
            # displays current joint angle
            angle = int(math.degrees(robot.relative_angles[i]))
            self.angle_labels.append(self.font.render(f"{angle}", True, settings.TEXT_COLOR))


    def update_labels(self, robot, screen):
        for i in range(len(self.inner_panels)):
            if i == 0:
                self.speed_label = self.font.render(f"{self.angle_speed}", True, settings.TEXT_COLOR)
            else:
                angle = int(math.degrees(robot.relative_angles[i-1]))
                self.angle_labels[i-1] = (self.font.render(f"{angle}", True, settings.TEXT_COLOR))


        
        for i in range(len(self.inner_panels)):
            #labels the angle speed
            if i == 0:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.speed_label, (center_x-12, center_y-12))

            #labels the joint angles for the remaining panels
            else:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.angle_labels[i-1], (center_x-12, center_y-12))

    # draws the angle controller
    def draw_controller(self, screen):

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
