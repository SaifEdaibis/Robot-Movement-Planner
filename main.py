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
                if self.world.angle_controller.angle_control_background.collidepoint(event.pos):
                    pass

                else:
                    #left click places a start position
                    if event.button == 1:
                        self.world.icons.start_pos = event.pos
                        self.world.icons.end_pos = None

                    # right click places an end position
                    else:
                        if self.world.icons.end_pos == None:
                            self.world.icons.end_pos = event.pos  
                        # another right click begins the movement of the arm  
                        else:
                            old_1, old_2, old_3 = self.planner.final_angles(self.world.icons.start_pos, self.world.robot, self.world, elbow_sign = 1)

                            
                            new_1, new_2, new_3 = self.planner.final_angles(self.world.icons.end_pos, self.world.robot, self.world, elbow_sign =1)
                            self.world.robot.Set_Angles(old_1, old_2, old_3)
                            self.path_list = self.world.robot.Route_Taker(new_1, new_2, new_3, self.display_front, self.world, self.planner)

                            if self.path_list is None:
                                new_1, new_2, new_3 = self.planner.final_angles(self.world.icons.end_pos, self.world.robot, self.world, elbow_sign=-1)
                                self.world.robot.Set_Angles(old_1, old_2, old_3)
                                self.path_list = self.world.robot.Route_Taker(new_1, new_2, new_3, self.display_front, self.world, self.planner)
                                

                    
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

    # The main_loop that draws and calls every repeated functions
    def main_loop(self):    
        while self.run:

            self.timing()

            self.process_events()
            
            self.display_front.screen.fill(settings.BACKGROUND_COLOR)

            self.display_front.draw_grid()

            self.display_front.draw_world(self.world)

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
                    pygame.time.delay(20)
                
            pygame.display.update()
            

Main = Application()
Main.main_loop()

pygame.quit()