import pygame
import math

pygame.init()

#--- Constants ------------------------------------------------------------------------------------------------------------------------------------------------------------

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

ANGLE_DISPLAY_PANEL_NUMBER = 3
INNER_PANEL_X = 40
INNER_PANEL_Y = 60
INNER_PANEL_DIMENSIONS = 50
INNER_PANEL_COLOR = (255,255,255)

TEXT_COLOR = (0,0,0)

# Icons
START_CIRCLE_COLOR = (128, 0, 0)
END_CIRCLE_COLOR = (0, 100, 0)

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
                            START_CIRCLE_COLOR,
                            self.start_pos,
                            JOINT_RADIUS,
                            width = 0
                        )

        # draws the green end icon
        if self.end_pos:
            pygame.draw.circle(
                                    screen,
                                    END_CIRCLE_COLOR,
                                    self.end_pos,
                                    JOINT_RADIUS,
                                    width = 0
                                )

#--- Path Planner -----------------------------------------------------------------------------------------------------------------------------------------

# This class detrmines the angles necescary to go from the start to end positions
class Path_Planner:
    def __init__(self):
        pass

    
    def final_angles(self, end_pos, robot):

        # stands for hypotenuse, this is the distance between the base and the 3rd joint. This cannot exceed 200 or the program will crash
        hypo = 250

        # how many times my program has tried to get a sutable angle
        tries = 1

        # this program picks a angle for the third joint to allow for the callculation of the other two. It also ensures the 3rd joint is within 200 pixels to prevent a program crash
        while hypo >= 200:

            #the target position
            final_x, final_y = end_pos

            # the angle for the third joint
            theta_3 = math.pi * ((1/32) * tries)

            #the positions of the third joint
            final_x = final_x - math.cos(theta_3) * robot.arm_lengths[-1]
            final_y = final_y + math.sin(theta_3) * robot.arm_lengths[-1]

            start_x, start_y = BASE_JOINT

            #the differinces between the base joint and third joint. CANNOT EXCEED 200
            delta_x = final_x - start_x
            delta_y = final_y - start_y

            tries += 1
            hypo = math.sqrt((delta_x ** 2) + (delta_y ** 2))

            # stops the loop if the arm goes all the way around
            if tries > 64:
                return
        
        #the angle between the first and second arm which is the relative_angle for the second joint
        inside_angle_b = math.acos(
                                ((delta_x ** 2) 
                                + (delta_y ** 2)
                                - (robot.arm_lengths[0] ** 2)
                                - (robot.arm_lengths[1] ** 2))
                                / (-2 * robot.arm_lengths[0] * robot.arm_lengths[1])
                           )

        # relative angle for the sceond joint
        relative_angle_2 = inside_angle_b

        # the angle between the first arm and the hypotnuse created from the base joint to the third joint
        inside_angle_a = math.acos(
                                        ( (robot.arm_lengths[1] ** 2)
                                        - (delta_x ** 2) 
                                        - (delta_y ** 2)
                                        - (robot.arm_lengths[0] ** 2))
                                        / (-2 * robot.arm_lengths[0] * math.sqrt((delta_x **2)+ (delta_y ** 2)))
                                   )

        # the angle between the hypotnuse created from the base joint to the third joint and the x axis
        base_angle_a = math.atan2(-delta_y,delta_x)

        # the joint angles for the first two joints
        joint_angle_1 = inside_angle_a + base_angle_a
        joint_angle_2 = relative_angle_2 - math.radians(180) + joint_angle_1

        return (joint_angle_1, joint_angle_2, theta_3)
        

#--- Robot -------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Robot:
    def __init__(self):

        self.joints = [None] * JOINT_NUMBERS
        self.base_joint =   BASE_JOINT

        # creates a list that records the angle of each joint
        self.joint_angles = [
            BASE_JOINT_ANGLE,
            SECOND_JOINT_ANGLE,
            THIRD_JOINT_ANGLE,
        ]

        # creates a list that records the angle of each arm realtive to the prevous arm. This is used to make arms position fixed
        self.relative_angles = [
            BASE_JOINT_ANGLE,

            math.radians((180 - math.degrees(BASE_JOINT_ANGLE)) 
            + math.degrees(SECOND_JOINT_ANGLE)),

            math.radians((180 - math.degrees(SECOND_JOINT_ANGLE))
            + math.degrees(THIRD_JOINT_ANGLE))
        ]

        # creates list that records starting arm lengths
        self.arm_lengths = [
            STARTING_ARM_LENGTH_ONE,
            STARTING_ARM_LENGTH_TWO,
            STARTING_ARM_LENGTH_THREE
        ]

        self.calculate_joint_pos()

    # calculates where each joint is based on angle and arm length
    def calculate_joint_pos(self):

        for joints in range(len(self.joints)):
            if joints == 0:

                self.joints[joints] = (self.base_joint)
            else:

                self.joints[joints] = ((
                    self.joints[joints-1][0] 
                    + math.cos(self.joint_angles[joints-1])*self.arm_lengths[joints-1], 

                    self.joints[joints-1][1] 
                    - math.sin(self.joint_angles[joints-1])*self.arm_lengths[joints-1]
                ))

    #draws the arms and joints of the robot
    def draw_robot(self, screen):

        for arm in range(len(self.joints)-1):

            # draws the robots arms
            pygame.draw.line(
                screen,
                ARM_COLOR,
                self.joints[arm],
                self.joints[arm+1], 
                ARM_WIDTH
            )

        for joint in range(len(self.joints)):

            #draws the robots outer joints
            pygame.draw.circle(
                screen,
                JOINT_COLOR,
                self.joints[joint],
                JOINT_RADIUS,
                width = 0
            )

            #draws the robots inner joints
            pygame.draw.circle(
                screen,
                ARM_COLOR,
                self.joints[joint],
                INNER_JOINT_RADIUS,
                width = 0
            )

    # the method changes the angle and relative angle of each joint and arm
    def change_angle(self, joint, angle):

        # limits arms from getting within 10 degrees of an another arm
        self.test_joint_angle = self.joint_angles[joint] - math.radians(angle)
        self.text_relative_angle = (180 - math.degrees(self.joint_angles[joint-1])) + math.degrees(self.test_joint_angle)

        if self.text_relative_angle < 10 or self.text_relative_angle > 350:
            return

        # changes joint angle
        self.joint_angles[joint] -= math.radians(angle)

        # changes realtive arm angle
        if joint == 0:
            self.relative_angles[joint] = self.joint_angles[joint]
        else:
            self.relative_angles[joint] = math.radians((180 - math.degrees(self.joint_angles[joint-1])) + math.degrees(self.joint_angles[joint]))
            
        #updates each joint angle based on the new relative and joint angle
        for joints in range(len(self.joint_angles)):
            if joints == 0:
                pass
            else:
                self.joint_angles[joints] = math.radians(math.degrees(self.relative_angles[joints]) - (180 - math.degrees(self.joint_angles[joints-1])))

        # calculates new joint positions
        self.calculate_joint_pos()

    # sets all 3 angles for the arm practically teleporting it 
    def Set_Angles(self, angle_1, angle_2, angle_3):
        new_angles = [angle_1, angle_2, angle_3]

        for i in range(len(self.joint_angles)):
            self.joint_angles[i] = new_angles[i]

            if i == 0:
                self.relative_angles[i] = self.joint_angles[i]
            else:
                self.relative_angles[i] = math.radians((180 - math.degrees(self.joint_angles[i-1])) + math.degrees(self.joint_angles[i]))

        self.calculate_joint_pos()

    # calculates all the angles between the start and end position creating a route for the arm
    def Route_Taker(self, new_1, new_2, new_3, display, world):
        new = [new_1, new_2, new_3]

        # stores all the angles for each joint
        angle_dictionary = {
            0: [],
            1: [],
            2: []
        }

        for i in range(len(self.joint_angles)):
            diff = (new[i] - self.joint_angles[i]) / 10
            angle_dictionary[i].append(self.joint_angles[i])
            for l in range(10):

                angle_dictionary[i].append(self.joint_angles[i] + diff * (l+1))

        return angle_dictionary
                        
#--- Controller --------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Angle_Controller():
    def __init__(self, robot):

        # creates the background rectangle for the controller
        self.angle_control_background = pygame.Rect(
            CONTROL_BACKGROUND_X,
            CONTROL_BACKGROUND_Y, 
            BACKGROUND_WIDTH, 
            BACKGROUND_HEIGHT
        )

        self.font = pygame.font.Font(None, 35)

        # this variable determines the rate of change of the joint angle
        self.angle_speed = 1

        #these lists hold the panels and the angle labels within the crontoller  
        self.inner_panels = []
        self.angle_labels = []

        # adds the first panel meant for displaying angle speed
        self.inner_panels.append(pygame.Rect(
            INNER_PANEL_X,
            INNER_PANEL_Y,
            INNER_PANEL_DIMENSIONS,
            INNER_PANEL_DIMENSIONS
        ))

        # adds the remaining panels alongside the respective angles they need to display
        for i in range(ANGLE_DISPLAY_PANEL_NUMBER):

            self.inner_panels.append(pygame.Rect(
                INNER_PANEL_X,
                INNER_PANEL_Y + 70*(i+1),
                INNER_PANEL_DIMENSIONS,
                INNER_PANEL_DIMENSIONS
            ))

            # displays current joint angle
            angle = int(math.degrees(robot.relative_angles[i]))
            self.angle_labels.append(self.font.render(f"{angle}", True, TEXT_COLOR))

    def update_labels(self, robot):
        for i in range(len(self.inner_panels)):
            if i == 0:
                self.speed_label = self.font.render(f"{self.angle_speed}", True, TEXT_COLOR)
            else:
                angle = int(math.degrees(robot.relative_angles[i-1]))
                self.angle_labels[i-1] = (self.font.render(f"{angle}", True, TEXT_COLOR))

    # draws the angle controller
    def draw_controller(self, screen, robot):

        # draws the background panel for the controller
        pygame.draw.rect(
            screen, 
            CONTROL_BACKGROUND_COLOR,
            self.angle_control_background
        )

        # draws the inner panels alongside their labels
        for i in range(len(self.inner_panels)):

            # draws each inner panel
            pygame.draw.rect(screen, INNER_PANEL_COLOR, self.inner_panels[i])

            #labels the angle speed
            if i == 0:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.speed_label, (center_x-12, center_y-12))

            #labels the joint angles for the remaining panels
            else:
                center_x, center_y = self.inner_panels[i].center
                screen.blit(self.angle_labels[i-1], (center_x-12, center_y-12))

#--- World ------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# owns everything on the screen
class World:
    def __init__(self):
        self.robot = Robot()
        self.angle_controller = Angle_Controller(self.robot)
        self.icons = Path_Icons()

#--- Main Screen ------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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

            start_pos = (
                0,
                GRID_DIMENSION * (line +1)
            )

            end_pos = (
                SCREEN_WIDTH, 
                GRID_DIMENSION * (line +1)
            )

            pygame.draw.line(
                self.screen,
                GRID_COLOR,
                start_pos,
                end_pos,
                GRID_LINE_WIDTH
            )

        # makes the coloumns for the grid
        for line in range(int((SCREEN_WIDTH / GRID_DIMENSION)-1)):

            start_pos = (
                GRID_DIMENSION * (line +1),
                0
            )
            end_pos = (
                GRID_DIMENSION * (line +1),
                SCREEN_HEIGHT
            )

            pygame.draw.line(
                self.screen,
                GRID_COLOR,
                start_pos,
                end_pos,
                GRID_LINE_WIDTH
            )

    #draws everything owned by the world
    def draw_world(self, world):
        
        world.icons.draw_icons(self.screen)

        world.robot.draw_robot(self.screen)

        world.angle_controller.update_labels(world.robot)
        world.angle_controller.draw_controller(self.screen, world.robot)


#--- Application ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Owns everything associated with the function of the main program
class Application:
    def __init__(self):
        self.display_front = Front_Display()
        self.world = World()
        self.run = True
        self.planner = Path_Planner()
        self.path_dictionary = None
        self.counter = 0

    # Controls the framerate of the program
    def timing(self):
        self.dt = self.display_front.clock.tick(FRAMERATE) / 1000

    # processess and responds to all major user inputs
    def process_events(self):

        for event in pygame.event.get():

            #allows the user to quit the game 
            if event.type == pygame.QUIT:                      
                self.run = False

            #allows the user to change the angle in each joint
            if event.type == pygame.MOUSEBUTTONDOWN:

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
                            old_1, old_2, old_3 = self.planner.final_angles(self.world.icons.start_pos, self.world.robot)

                            new_1, new_2, new_3 = self.planner.final_angles(self.world.icons.end_pos, self.world.robot)
                            self.world.robot.Set_Angles(old_1, old_2, old_3)

                            self.path_dictionary = self.world.robot.Route_Taker(new_1, new_2, new_3, self.display_front, self.world)

                    
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

    # The main_loop that draws and calls every repeated functions
    def main_loop(self):    
        while self.run:

            self.timing()

            self.process_events()
            
            self.display_front.screen.fill(BACKGROUND_COLOR)

            self.display_front.draw_grid()

            self.display_front.draw_world(self.world)

            if self.path_dictionary:
                self.world.robot.Set_Angles(
                    self.path_dictionary[0][self.counter],
                    self.path_dictionary[1][self.counter],
                    self.path_dictionary[2][self.counter],
                    )

                if self.counter == 10:
                    self.path_dictionary = None
                    self.counter = 0
                else:
                    self.counter += 1
                pygame.time.delay(200)
            
                
            pygame.display.update()
            

Main = Application()
Main.main_loop()

pygame.quit()