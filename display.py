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

        world.angle_controller.draw_controller(self.screen)
        world.angle_controller.update_labels(world.robot, self.screen, 0)

        print(type(world.path_controller), world.path_controller.__class__.__mro__)
        
        world.path_controller.draw_controller(self.screen)
        world.path_controller.update_function_label(self.screen, 1)

        world.start_angle_display.draw_controller(self.screen)
        
        world.end_angle_display.draw_controller(self.screen)
        
        world.status_menu.draw_controller(self.screen)

        world.draw_delete_zone(self.screen)
        world.delete_object()
        
        world.spawn_obstacles()

        world.status_menu.draw_controller(self.screen, 2)
        world.status_menu.update_menu(self.screen)

        world.end_angle_display.update_labels(world.robot, self.screen, 3, world.end_angle_display.label_list)
        world.start_angle_display.update_labels(world.robot, self.screen, 2, world.start_angle_display.label_list)
                    


        for i in range(settings.JOINT_NUM):
                    if i == 0:
                        start_angle = 0
                        end_angle = world.robot.joint_angles[0]
                    else:
                        start_angle = world.robot.joint_angles[i-1] + math.pi
                        raw_diff = (world.robot.joint_angles[i] - start_angle) % (2 * math.pi)

                        end_angle = start_angle + raw_diff

                    if end_angle < start_angle:
                        start_angle, end_angle = end_angle, start_angle

                    x, y = world.robot.joints[i]
                    center = (
                        x - (settings.ARC_DIAMETER/2),
                        y - (settings.ARC_DIAMETER/2),
                        settings.ARC_DIAMETER,
                        settings.ARC_DIAMETER
                    )
            
                    world.arcs[i].update_arc(start_angle, end_angle, center)
                    world.arcs[i].draw_arc(self.screen, i)

        for i in range(len(world.obstacles)):
            world.obstacles[i].draw_obstacle(self.screen)
