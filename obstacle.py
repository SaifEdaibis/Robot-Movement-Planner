import math
import pygame
import heapq

import settings

#--- Obstacle --------------------------------------------------------------------------------------------------------------------------------------------

#creates obstacles for the arm
class Obstacle:
    def __init__(self, num):
        self.rect = pygame.Rect(
            settings.OBSTACLE_START_X + num * 100,
            settings.OBSTACLE_START_Y,
            settings.OBSTACLE_DIMENSIONS,
            settings.OBSTACLE_DIMENSIONS
        )

        self.selected_status = False

    # draws the obstacle
    def draw_obstacle(self, screen):
        pygame.draw.rect(screen, settings.OBSTACLE_COLOR , self.rect)

    # checks if the obstacle is within the boundaries of the game
    def boundary_check(self, x_change, y_change):       
        if self.rect.left + x_change <= 0:
            x_change = -self.rect.left

        elif self.rect.right + x_change >= settings.SCREEN_WIDTH:
            x_change = settings.SCREEN_WIDTH - self.rect.right 
        
        if self.rect.top + y_change <= 0:
            y_change = -self.rect.top
        
        elif self.rect.bottom + y_change >= settings.SCREEN_HEIGHT:
            y_change = settings.SCREEN_HEIGHT - self.rect.bottom

        return x_change, y_change

    # prevents overlap between the obstacles
    def collision_check(self, world, x_change=0, y_change=0):

        for obj in world.obstacles:
            if obj is self:
                continue

            # --- check x movement on its own ---
            x_rect = self.rect.move(x_change, 0)
            if x_rect.colliderect(obj.rect):
                if x_change > 0:
                    x_change = min(x_change, obj.rect.left - self.rect.right)
                elif x_change < 0:
                    x_change = max(x_change, obj.rect.right - self.rect.left)

            # --- check y movement on its own ---
            y_rect = self.rect.move(0, y_change)
            if y_rect.colliderect(obj.rect):
                if y_change > 0:
                    y_change = min(y_change, obj.rect.top - self.rect.bottom)
                elif y_change < 0:
                    y_change = max(y_change, obj.rect.bottom - self.rect.top)

        return x_change, y_change    
