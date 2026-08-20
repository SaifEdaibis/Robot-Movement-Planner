import copy

import pygame
import math
import heapq

from path import Path_Planner
from world import World
from world import Path_Icons
from display import Front_Display
import settings 

pygame.init()

#--- Application ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Owns everything associated with the function of the main program
class Application:
    def __init__(self):
        self.display_front = Front_Display()
        self.world = World()
        self.run = True
        self.planner = Path_Planner()
        self.path_list = None
        self.counter = 0

    # Controls the framerate of the program
    def timing(self):
        self.dt = self.display_front.clock.tick(settings.FRAMERATE) / 1000

    # processess and responds to all major user inputs
    def process_events(self):

        for event in pygame.event.get():

            #allows the user to quit the game 
            if event.type == pygame.QUIT:                      
                self.run = False

            #processes user clicks
            if event.type == pygame.MOUSEBUTTONDOWN:

                #allows user to select obstacles
                if event.button == 1:
                    for item in self.world.obstacles:
                        if item.rect.collidepoint(event.pos):
                            item.selected_status = True

                #cursor can not be on the angle controller
                if self.world.angle_controller.angle_control_background.collidepoint(event.pos) or self.world.path_controller.angle_control_background.collidepoint(event.pos):
                    pass
                else:
                    #left click places a start position
                    if self.world.path_controller.inner_panels_bool[0] == True and self.world.path_controller.inner_panels_bool[1] == False:
                        self.world.icons.start_pos = event.pos
                        self.world.icons.end_pos = None
                        self.world.path_controller.inner_panels_bool[1] = True
                        self.world.path_controller.status = "end pos"

                    elif self.world.path_controller.inner_panels_bool[1] == True:
                        if self.world.icons.end_pos == None:
                            self.world.icons.end_pos = event.pos 
                            
                            self.world.path_controller.status = "loading"
                            self.world.path_controller.counter = 0

                if self.world.path_controller.inner_panels[0].collidepoint(event.pos):
                    if self.world.path_controller.status == "paused":
                        self.world.path_controller.inner_panels_bool[0] = True 
                        self.world.path_controller.status = "start pos"

                        
                for num, panel in enumerate(self.world.angle_controller.inner_panels):

                    #changes the angle speed
                    if num == 0:
                        if panel.collidepoint(event.pos):      
                            if event.button == 1:
                                self.world.angle_controller.angle_speed += 1
                            else:
                                self.world.angle_controller.angle_speed += -1

                    #changes the individual joint angle
                    else:
                        if panel.collidepoint(event.pos):      
                            if event.button == 1:
                                self.world.robot.change_angle(num-1, self.world.angle_controller.angle_speed)
                            else:
                                self.world.robot.change_angle(num-1, -self.world.angle_controller.angle_speed)

            # processses the movement of obstacles
            if event.type == pygame.MOUSEMOTION:            
                for item in self.world.obstacles:
                    if item.selected_status == True:
                        x_change, y_change = event.rel

                        x_change, y_change = item.boundary_check(x_change, y_change)

                        x_change, y_change = item.collision_check(self.world, x_change, y_change)

                        item.rect.move_ip(x_change, y_change)

            # processess obstacle release
            for item in self.world.obstacles:                        
                if item.selected_status == True:
                    if event.type == pygame.MOUSEBUTTONUP:
                        if event.button == 1:
                            item.selected_status = False

    def find_route(self):
        
        if self.world.icons.start_pos and self.world.icons.end_pos:
            self.path_list = None

            start_pos = copy.copy(self.world.icons.start_pos)
            end_pos = copy.copy(self.world.icons.end_pos)
            self.world.icons.start_pos = None
            self.world.icons.end_pos = None
            

            old_list = self.planner.final_angles(start_pos, self.world.robot, self.world, elbow_sign = 1)
            new_list = self.planner.final_angles(end_pos, self.world.robot, self.world, elbow_sign =1)

            if old_list == None or new_list == None:
                return
            else:
                old_1, old_2, old_3 = old_list
                new_1, new_2, new_3 = new_list

            self.path_list = self.world.robot.Route_Taker(old_1, old_2, old_3, new_1, new_2, new_3, self.display_front, self.world, self.planner)

            if self.path_list is None:
                new_list = self.planner.final_angles(end_pos, self.world.robot, self.world, elbow_sign=-1)

                if new_list == None:
                    return
                else:
                    new_1, new_2, new_3 = new_list

                self.path_list = self.world.robot.Route_Taker(old_1, old_2, old_3, new_1, new_2, new_3, self.display_front, self.world, self.planner)

            if self.path_list is None:
                return

            self.world.robot.Set_Angles(old_1, old_2, old_3)

            self.world.path_controller.inner_panels_bool[0] = False
            self.world.path_controller.inner_panels_bool[1] = False

            self.world.path_controller.status = "paused"

    # The main_loop that draws and calls every repeated functions
    def main_loop(self):

        while self.run:

            self.timing()

            self.process_events()
           
            
            self.display_front.screen.fill(settings.BACKGROUND_COLOR)

            pygame.draw.circle(
                        self.display_front.screen,
                        settings.BACK_CIRCLE_COLOR,
                        settings.BASE_JOINT,
                        settings.STARTING_ARM_LENGTH * 3,
                        width = 0
                    )

            self.display_front.draw_grid()

            self.display_front.draw_world(self.world)
                
            pygame.display.update()

            if self.world.path_controller.counter == 0:
                self.find_route()

            if self.path_list:
                if self.counter == len(self.path_list):
                    self.path_list = None
                    self.counter = 0

                else:
                    self.world.robot.Set_Angles(
                        angle_1  = math.radians(self.path_list[self.counter][0] * settings.RESOLUTION ),
                        angle_2  = math.radians(self.path_list[self.counter][1] * settings.RESOLUTION ),
                        angle_3  = math.radians(self.path_list[self.counter][2] * settings.RESOLUTION )
                    )
                    self.counter += 1
                    pygame.time.delay(50)

            

Main = Application()
Main.main_loop()

pygame.quit()