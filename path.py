import math
import pygame
import heapq

# Screen Info
SCREEN_WIDTH = 600              ### pixels
SCREEN_HEIGHT = 600
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

# Icons Info
START_CIRCLE_COLOR = (128, 0, 0)
END_CIRCLE_COLOR = (0, 100, 0)

# Obstacle Info
OBSTACLE_COLOR = (135, 206, 235)
OBSTACLE_START_X = 180
OBSTACLE_START_Y = 400
OBSTACLE_DIMENSIONS = 50

# Route Info
RESOLUTION = 5
COLLISION_PADDING = ARM_WIDTH
JOINT_COLLISION_PADDING = JOINT_RADIUS * 2
WEIGHT = 1.5

#--- Path Planner -----------------------------------------------------------------------------------------------------------------------------------------

# This class detrmines the angles necescary to go from the start to end positions
class Path_Planner:
    def __init__(self):
        self.node_dictionary = {}
        self.closed_set = set()

    
    def final_angles(self, end_pos, robot, elbow_sign):

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
        inside_angle_b = elbow_sign * math.acos(
                                ((delta_x ** 2) 
                                + (delta_y ** 2)
                                - (robot.arm_lengths[0] ** 2)
                                - (robot.arm_lengths[1] ** 2))
                                / (-2 * robot.arm_lengths[0] * robot.arm_lengths[1])
                           )

        # relative angle for the sceond joint
        relative_angle_2 = inside_angle_b

        # the angle between the first arm and the hypotnuse created from the base joint to the third joint
        inside_angle_a = elbow_sign * math.acos(
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

    def position_validifier(self, node_tuple, robot, world):
        i, j, k = node_tuple

        if (i, j, k) in self.node_dictionary:
            return self.node_dictionary[(i, j, k)]
        
        angle_1 = math.radians(i * RESOLUTION)
        angle_2 = math.radians(j * RESOLUTION)
        angle_3 = math.radians(k * RESOLUTION)
        self.test_joint_pos = robot.calculate_joint_pos(angle_1, angle_2, angle_3)

        for obstacle in world.obstacles:

            for joint_pos in self.test_joint_pos:
                padded_rect = obstacle.rect.inflate(JOINT_COLLISION_PADDING, JOINT_COLLISION_PADDING)
                if padded_rect.collidepoint(joint_pos):
                    self.node_dictionary[(i, j, k)] = False
                    return False

            for arm in range(len(self.test_joint_pos) - 1):
                padded_rect = obstacle.rect.inflate(COLLISION_PADDING, COLLISION_PADDING)
                self.collision = bool(obstacle.rect.clipline(self.test_joint_pos[arm],self.test_joint_pos[arm + 1]))

                if self.collision:
                    self.node_dictionary[(i, j, k)] = False
                    return False

        self.node_dictionary[(i, j, k)] = True
        return True