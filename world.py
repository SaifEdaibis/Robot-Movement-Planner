import math
import pygame
import heapq

from robot import Robot
from controller import Angle_Controller
from obstacle import Obstacle
from Arcs import Angle_Arc
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
                    settings.CONTROL_BACKGROUND_Y + 140, 
                    settings.BACKGROUND_WIDTH * 0.75, 
                    settings.BACKGROUND_HEIGHT)

        self.start_angle_display = Angle_Controller(
                            settings.CONTROL_BACKGROUND_X,
                            settings.CONTROL_BACKGROUND_Y + 280, 
                            settings.BACKGROUND_WIDTH * 0.75, 
                            settings.BACKGROUND_HEIGHT)

        self.end_angle_display = Angle_Controller(
                                    settings.CONTROL_BACKGROUND_X,
                                    settings.CONTROL_BACKGROUND_Y + 420, 
                                    settings.BACKGROUND_WIDTH * 0.75, 
                                    settings.BACKGROUND_HEIGHT)
        
        self.angle_controller.panel_maker(self.robot, settings.ANGLE_DISPLAY_PANEL_NUMBER, row = 0, excepetion=1)
        self.angle_controller.set_labels(self.robot, settings.ANGLE_DISPLAY_PANEL_NUMBER)

        self.path_controller.function_panel_maker(self.robot, 2, 1)
        self.path_controller.update_panels(2)

        self.start_angle_display.panel_maker(self.robot, 2, 2)
        self.end_angle_display.panel_maker(self.robot, 2, 3)

        self.icons = Path_Icons()

        self.obstacles = []
        self.obstacles.append(Obstacle())

        self.arcs = []
        for i in range(settings.JOINT_NUM):
            if i == 0:
                start_angle = 0
                end_angle = self.robot.joint_angles[0]
            else:
                start_angle = self.robot.joint_angles[i-1] + math.pi
                raw_diff = (self.robot.joint_angles[i] - start_angle) % (2 * math.pi)
                if raw_diff > math.pi:
                    raw_diff -= 2 * math.pi   # take the shorter way around
                end_angle = start_angle + raw_diff

            if end_angle < start_angle:
                start_angle, end_angle = end_angle, start_angle

            x, y = self.robot.joints[i]
            center = (
                x - (settings.ARC_DIAMETER/2),
                y - (settings.ARC_DIAMETER/2),
                settings.ARC_DIAMETER,
                settings.ARC_DIAMETER
            )

            self.arcs.append(Angle_Arc(start_angle, end_angle, center))

            self.delete_zone = pygame.Rect(
                                    settings.OBSTACLE_START_X,
                                    settings.OBSTACLE_START_Y + settings.DELETE_SPACING, 
                                    settings.OBSTACLE_DIMENSIONS, 
                                    settings.OBSTACLE_DIMENSIONS)

    def spawn_obstacles(self):
        test_object = Obstacle()
        test_one, test_two = test_object.collision_check(self)

        if test_one is not None and test_two is not None:
            self.obstacles.append(Obstacle())

    def draw_delete_zone(self, screen):
        self.font = pygame.font.Font(None, 30)
        pygame.draw.rect(screen, settings.DELETE_ZONE_COLOR, self.delete_zone)
        top_left = self.delete_zone.topleft
        center_pos = (top_left[0] + 4,
                      top_left[1] + settings.OBSTACLE_DIMENSIONS *(5/16))

        self.delete_label = self.font.render(f"DEL", True, settings.TEXT_COLOR)
        screen.blit(self.delete_label, center_pos)

    def delete_object(self):
        obj_list = []
        for obj in range(len(self.obstacles)):
            if self.obstacles[obj].rect.colliderect(self.delete_zone):
                obj_list.append(obj)

        for i in reversed(obj_list):
            del self.obstacles[i]