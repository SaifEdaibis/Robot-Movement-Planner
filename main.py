import pygame
import math

pygame.init()

#--- Constants -------------------------------------------------------------------------

# Screen Info
SCREEN_WIDTH = 400              ### pixels
SCREEN_HEIGHT = SCREEN_WIDTH
BACKGROUND_COLOR = (255, 255, 255)

# Timing Info
FRAMERATE = 60

# Grid Info
GRID_DIMENSION = 40             ### pixels
GRID_COLOR = (0, 0, 0)
GRID_LINE_WIDTH = 2             ### pixels

# Robot Info
BASE_JOINT = (200, 350)    ### pixels

STARTING_ANGLE_ONE = math.pi/2
STARTING_ANGLE_TWO = math.pi/2
STARTING_ANGLE_THREE = math.pi/2

STARTING_ARM_LENGTH_ONE = 100
STARTING_ARM_LENGTH_TWO = 100
STARTING_ARM_LENGTH_THREE = 100

JOINT_NUMBERS = 4
ARM_COLOR = (0, 0, 0)
JOINT_COLOR = (212, 175, 55)
JOINT_RADIUS = 9
INNER_JOINT_RADIUS = 5
ARM_WIDTH = 7                  ### pixels


#--- Robot -----------------------------------------------------------------------------------------

class Robot:
    def __init__(self):

        self.joints = [None] * JOINT_NUMBERS
        self.base_joint =   BASE_JOINT

        self.joint_angles = [
            STARTING_ANGLE_ONE,
            STARTING_ANGLE_TWO,
            STARTING_ANGLE_THREE,
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
                self.joints[joints] = (BASE_JOINT)
            else:
                    self.joints[joints] = ((self.joints[joints-1][0] + math.cos(self.joint_angles[joints-1])*self.arm_lengths[joints-1], 
                                            self.joints[joints-1][1]- math.sin(self.joint_angles[joints-1])*self.arm_lengths[joints-1]))

    def draw_robot(self, screen):

        for arm in range(len(self.joints)-1):
            pygame.draw.line(screen, ARM_COLOR, self.joints[arm], self.joints[arm+1], ARM_WIDTH)

        for joint in range(len(self.joints)):
            pygame.draw.circle(screen, JOINT_COLOR, self.joints[joint], JOINT_RADIUS, width = 0)
            pygame.draw.circle(screen, ARM_COLOR, self.joints[joint], INNER_JOINT_RADIUS, width = 0)

    def move_robot(self, joint, angle):
        self.joint_angles[joint] = angle

        self.calculate_joint_pos()


#--- World -----------------------------------------------------------------------------------------

# owns everything on the screen
class World:
    def __init__(self):
        self.robot = Robot()

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

    # The main_loop that draws and calls every repeated functions
    def main_loop(self):    
        x = 0
        while self.run:

            if x == 0:
                Main.world.robot.move_robot(0, 1)
                Main.world.robot.move_robot(1, 3)
                Main.world.robot.move_robot(2, 0.5)
                print(Main.world.robot.joints)
            else:
                pass

            x += 1
            
            self.timing()

            self.process_events()
            
            self.display_front.screen.fill(BACKGROUND_COLOR)

            self.display_front.draw_grid()

            self.display_front.draw_world(Main.world)
    
            pygame.display.update()
            



Main = Application()
Main.main_loop()



pygame.quit()