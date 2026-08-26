import pygame
import settings

class Angle_Arc():

    def __init__(self, start, end, center):
        self.start_angle = start
        self.end_angle = end
        self.rect = center

    def update_arc(self, start, end, center):
        self.start_angle = start
        self.end_angle = end
        self.rect = center

    def draw_arc(self, screen, arc):
        pygame.draw.arc(
            screen,
                        settings.ARC_DIC[arc],
                        self.rect,
                        self.start_angle,
                        self.end_angle,
                        settings.ARC_WIDTH
        )
            