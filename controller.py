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

        self.status = None
        self.counter = None
        self.label_list = None
        self.angle_labels = []
        self.label_list = [0, 0, 0]

        self.font = pygame.font.Font(None, 30)

        # this variable determines the rate of change of the joint angle
        self.angle_speed = 1

    def panel_maker(self, robot, panel, row = 0, excepetion = None):

        #these lists hold the panels and the angle labels within the crontoller  
        self.inner_panels = []
        self.left_panel = []
        self.inner_panels_bool = []

        # adds the panels
        for i in range(panel+1):

            if excepetion:
                placeholder = i + 1
            else:
                placeholder = i

            if placeholder != panel + 1:
                self.left_panel.append(pygame.Rect(
                    settings.INNER_PANEL_X + 70*(placeholder),
                    settings.INNER_PANEL_Y + row * 140,
                    settings.LEFT_PANEL_X,
                    settings.INNER_PANEL_DIMENSIONS
                ))

            self.inner_panels.append(pygame.Rect(
                            settings.INNER_PANEL_X + 70*(i),
                            settings.INNER_PANEL_Y + row * 140,
                            settings.INNER_PANEL_DIMENSIONS,
                            settings.INNER_PANEL_DIMENSIONS
                        ))

            self.inner_panels_bool.append(None)

    def function_panel_maker(self, robot, panel, row = 0):
        #these lists hold the panels and the angle labels within the crontoller  
                self.inner_panels = []
                self.inner_panels_bool = []
                self.status = "paused"
                self.left_panel = None

                self.inner_panels.append(pygame.Rect(
                                                    settings.INNER_PANEL_X,
                                                    settings.INNER_PANEL_Y + row * 140,
                                                    settings.INNER_FUNCTION_PANEL_WIDTH,
                                                    settings.INNER_FUNCTION_PANEL_HEIGHT
                                                ))
        
                # adds the panels
                for i in range(panel+1):
                    self.inner_panels_bool.append(None)

    def update_function_label(self, screen, row):

            self.font = pygame.font.Font(None, 30)
            position = self.angle_control_background.topleft
            main_label_pos = (position[0] + 10, position[1] + 10)
            self.main_label = self.font.render(f"{settings.CONTROLLER_DIC[row]}", True, settings.MAIN_TEXT_COLOR)
            screen.blit(self.main_label, main_label_pos)
            
            self.font = pygame.font.Font(None, settings.DIC[self.status][1])
            self.function_label = self.font.render(f"{settings.DIC[self.status][0]}", True, settings.FUNCTION_TEXT_COLOR)
            center_x, center_y = self.inner_panels[0].center
            width, height = self.font.size(f"{settings.DIC[self.status][0]}")
            screen.blit(
                self.function_label,
                (center_x - (width/2),
                 center_y- (height/2))
            )
        
    def update_panels(self, panel):

        # updates the panels to reflect the current color
        for i in range(panel):
            self.inner_panels_bool[i] = False

    def set_labels(self, robot, panel):
        self.angle_labels = []

        # adds the remaining angles they need to display
        for i in range(panel):
        
            # displays current joint angle
            angle = int(math.degrees(robot.relative_angles[i]))
            self.angle_labels.append(self.font.render(f"{angle}", True, settings.TEXT_COLOR))

    def update_labels(self, robot, screen, row, list = None):

        position = self.angle_control_background.topleft
        main_label_pos = (position[0] + 10, position[1] + 10)
        self.main_label = self.font.render(f"{settings.CONTROLLER_DIC[row]}", True, settings.MAIN_TEXT_COLOR)
        screen.blit(self.main_label, main_label_pos)
    
        if not list:
            for i in range(len(self.inner_panels)):
                if i == 0:
                    self.speed_label = self.font.render(f"{self.angle_speed}", True, settings.TEXT_COLOR)
                else:
                    angle = int(math.degrees(robot.relative_angles[i-1]))
                    self.angle_labels[i-1] = (self.font.render(f"{angle}", True, settings.TEXT_COLOR))
        else:

            self.angle_labels.clear()
            for i in range(len(self.inner_panels)):
                angle = int(math.degrees(list[i]))
                self.angle_labels.append(self.font.render(f"{angle}", True, settings.TEXT_COLOR))
        
        for i in range(len(self.inner_panels)):
            center_x, center_y = self.inner_panels[i].center
            if i == 0 and not list:
                width, height = self.speed_label.get_size()
            elif not list:
                width, height = self.angle_labels[i-1].get_size()
            else:
                width, height = self.angle_labels[i].get_size()
                
            label_pos = (
                center_x - (width/2),
                center_y- (height/2)
            )

            #labels the angle speed
            if not list:
                if i == 0:
                    screen.blit(self.speed_label, label_pos)

                #labels the joint angles for the remaining panels
                else:
                    screen.blit(self.angle_labels[i-1], label_pos)
            else:
                screen.blit(self.angle_labels[i], label_pos)

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

            if self.status:
                color = settings.COLOR_DIC[self.status]
            else:
                color = settings.INNER_PANEL_COLOR

            # draws each inner panel
            pygame.draw.rect(screen, color, self.inner_panels[i])

        if self.left_panel:
            for i in range(len(self.left_panel)):
                pygame.draw.rect(screen, settings.ARC_DIC[i], self.left_panel[i])
