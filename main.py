import pygame
import math

pygame.init()

#--- Constants -------------------------------------------------------------------------

# Screen Info
SCREEN_WIDTH = 600              ### pixels
SCREEN_HEIGHT = SCREEN_WIDTH * 0.8
BACKGROUND_COLOR = (255, 255, 255)

# Timing Info
FRAMERATE = 60

# Grid Info
GRID_DIMENSION = 40             ### pixels
GRID_COLOR = (0, 0, 0)
GRID_LINE_WIDTH = 2             ### pixels

# Robot Info
BASE_JOINT = (300, 350)         ### pixels

BASE_JOINT_ANGLE = math.pi/2
SECOND_JOINT_ANGLE = math.pi/2
THIRD_JOINT_ANGLE = math.pi/2

STARTING_ARM_LENGTH_ONE = 100
STARTING_ARM_LENGTH_TWO = 100
STARTING_ARM_LENGTH_THREE = 100

JOINT_NUMBERS = 4
ARM_COLOR = (0, 0, 0)
JOINT_COLOR = (212, 175, 55)
JOINT_RADIUS = 9
INNER_JOINT_RADIUS = 5
ARM_WIDTH = 7                  ### pixels

# Angle Controller
CONTROL_BACKGROUND_X = 20
CONTROL_BACKGROUND_Y = 40
BACKGROUND_WIDTH = 90
BACKGROUND_HEIGHT = 300
CONTROL_BACKGROUND_COLOR = (128,128,128)


INNER_PANEL_X = 40
INNER_PANEL_Y = 60
INNER_PANEL_DIMENSIONS = 50
INNER_PANEL_COLOR = (255,255,255)

TEXT_COLOR = (0,0,0)


#--- Robot -----------------------------------------------------------------------------------------

class Robot:
    def __init__(self):

        self.joints = [None] * JOINT_NUMBERS
        self.base_joint =   BASE_JOINT

        self.joint_angles = [
            BASE_JOINT_ANGLE,
            SECOND_JOINT_ANGLE,
            THIRD_JOINT_ANGLE,
        ]

        self.arm_lengths = [
            STARTING_ARM_LENGTH_ONE,
            STARTING_ARM_LENGTH_TWO,
            STARTING_ARM_LENGTH_THREE
        ]

        self.calculate_joint_pos()

    def calculate_joint_pos(self):

        for joints in range(JOINT_NUMBERS):
            if joints == 0:
                self.joints[joints] = (self.base_joint)
            else:
                self.joints[joints] = ((self.joints[joints-1][0] + math.cos(self.joint_angles[joints-1])*self.arm_lengths[joints-1], 
                                        self.joints[joints-1][1]- math.sin(self.joint_angles[joints-1])*self.arm_lengths[joints-1]))

    def draw_robot(self, screen):

        for arm in range(len(self.joints)-1):
            pygame.draw.line(screen, ARM_COLOR, self.joints[arm], self.joints[arm+1], ARM_WIDTH)

        for joint in range(len(self.joints)):
            pygame.draw.circle(screen, JOINT_COLOR, self.joints[joint], JOINT_RADIUS, width = 0)
            pygame.draw.circle(screen, ARM_COLOR, self.joints[joint], INNER_JOINT_RADIUS, width = 0)

    def change_angle(self, joint, angle):
        self.joint_angles[joint] -= math.radians(angle)

        self.calculate_joint_pos()

class Angle_Controller():
    def __init__(self, robot):
        self.angle_control_background = pygame.Rect(CONTROL_BACKGROUND_X, CONTROL_BACKGROUND_Y, BACKGROUND_WIDTH, BACKGROUND_HEIGHT)
        
        self.font = pygame.font.Font(None, 35)
        self.angle_speed = 1

        self.inner_panels = []
        self.angle_labels = []

        self.inner_panels.append(pygame.Rect(INNER_PANEL_X, INNER_PANEL_Y, INNER_PANEL_DIMENSIONS, INNER_PANEL_DIMENSIONS))

        for i in range(3):
            self.inner_panels.append(pygame.Rect(INNER_PANEL_X, INNER_PANEL_Y + 70*(i+1), INNER_PANEL_DIMENSIONS, INNER_PANEL_DIMENSIONS))
            angle = int(math.degrees(robot.joint_angles[i]))
            self.angle_labels.append(self.font.render(f"{angle}", True, TEXT_COLOR))



    def draw_controller(self, screen, robot):
        pygame.draw.rect(screen, CONTROL_BACKGROUND_COLOR , self.angle_control_background)

        for i in range(4):
            pygame.draw.rect(screen, INNER_PANEL_COLOR, self.inner_panels[i])

            if i == 0:
                self.speed_label = self.font.render(f"{self.angle_speed}", True, TEXT_COLOR)

                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.speed_label, (center_x-12, center_y-12))
            else:
                angle = int(math.degrees(robot.joint_angles[i-1]))
                self.angle_labels[i-1] = (self.font.render(f"{angle}", True, TEXT_COLOR))

                
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.angle_labels[i-1], (center_x-12, center_y-12))

#--- World -----------------------------------------------------------------------------------------

# owns everything on the screen
class World:
    def __init__(self):
        self.robot = Robot()
        self.angle_controller = Angle_Controller(self.robot)

#--- Main Screen -----------------------------------------------------------------------------------------

# Owns the screen and display and all associated functions
class Front_Display:
    def __init__(self):

        # Sets up the main screen
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption('Robotic Arm Movement Planner')

        # Sets up the screen timing
        self.clock = pygame.time.Clock()


    # makes the background grid for the screen
    def draw_grid(self):

        # makes the rows for the grid
        for line in range(int((SCREEN_HEIGHT / GRID_DIMENSION)-1)):
            start_pos = (0, GRID_DIMENSION * (line +1))
            end_pos = (SCREEN_WIDTH, GRID_DIMENSION * (line +1) )
            pygame.draw.line(self.screen, GRID_COLOR, start_pos, end_pos, GRID_LINE_WIDTH)

        # makes the coloumns for the grid
        for line in range(int((SCREEN_WIDTH / GRID_DIMENSION)-1)):
            start_pos = (GRID_DIMENSION * (line +1), 0)
            end_pos = ( GRID_DIMENSION * (line +1), SCREEN_HEIGHT)
            pygame.draw.line(self.screen, GRID_COLOR, start_pos, end_pos, GRID_LINE_WIDTH)

    def draw_world(self, world):
        world.robot.draw_robot(self.screen)
        world.angle_controller.draw_controller(self.screen, world.robot)

#--- Application -----------------------------------------------------------------------------------------------

# Owns everything associated with the function of the main program
class Application:
    def __init__(self):
        self.display_front = Front_Display()
        self.world = World()
        self.run = True

    # Controls the framerate of the program
    def timing(self):
        self.dt = self.display_front.clock.tick(FRAMERATE) / 1000

    # processess and responds to all major user inputs
    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:                      
                self.run = False

            if event.type == pygame.MOUSEBUTTONDOWN:
                    for num, panel in enumerate(self.world.angle_controller.inner_panels):
                        if num == 0:
                            if panel.collidepoint(event.pos):      
                                if event.button == 1:
                                    self.world.angle_controller.angle_speed += 1
                                else:
                                    self.world.angle_controller.angle_speed += -1
                        else:
                            if panel.collidepoint(event.pos):      
                                if event.button == 1:
                                    self.world.robot.change_angle(num-1, self.world.angle_controller.angle_speed)
                                else:
                                    self.world.robot.change_angle(num-1, -self.world.angle_controller.angle_speed)

    # The main_loop that draws and calls every repeated functions
    def main_loop(self):    
        while self.run:
            
            self.timing()

            self.process_events()
            
            self.display_front.screen.fill(BACKGROUND_COLOR)

            self.display_front.draw_grid()

            self.display_front.draw_world(Main.world)
    
            pygame.display.update()
            



Main = Application()
Main.main_loop()



pygame.quit()