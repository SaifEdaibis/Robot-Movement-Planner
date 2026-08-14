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

#--- Helper Functions -------------------------------------------------------------------------------------------------------------------------------------------------------------------

def relative_angles_from_tuple(node_tuple):
    i, j, k = [x * RESOLUTION for x in node_tuple]
    rel_2 = (180 - i) + j
    rel_3 = (180 - j) + k
    return rel_2 % 360, rel_3 % 360

# converts degrees into node values
def norm_deg(rad):
    return round(math.degrees(rad) % 360)

# checks the distance between the node and the final node to help determine the cost
def node_cost_check(node, final_node):

    distance = math.sqrt(
        ((final_node[0] - node[0]) **2)
        + ((final_node[1] - node[1]) **2)
        + ((final_node[2] - node[2]) **2)
    )

    return distance

# determines the neighboring nodes and sets them as possible options
def node_neighbor(node_tuple, planner, robot, world):
    node_options_list = []

    key = {
        0: [1, -1, 0, 0, 0, 0],
        1: [0, 0, 1, -1, 0, 0],
        2: [0, 0, 0, 0, 1, -1],
    }

    for i in range(6):
            test_tuple = (node_tuple[0] + key[0][i], node_tuple[1] + key[1][i], node_tuple[2] + key[2][i])
            if  test_tuple in planner.closed_set:
                continue
            rel_2, rel_3 = relative_angles_from_tuple(test_tuple)
            if not (5 <= rel_2 <= 355 and 5 <= rel_3 <= 355):
                continue
            if planner.position_validifier(test_tuple, robot, world):
                node_options_list.append(test_tuple)
    return node_options_list


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

        self.Set_Joint_pos(
            self.calculate_joint_pos()
        )

    # calculates where each joint is based on angle and arm length
    def calculate_joint_pos(self, angle_1 = None, angle_2 = None, angle_3 = None):
        self.temp_joints = [None] * len(self.joints)

        if angle_1 is not None:
            self.temp_joint_angles = [angle_1, angle_2, angle_3]
        else:
            self.temp_joint_angles = self.joint_angles

        for joints in range(len(self.joints)):
            if joints == 0:

                self.temp_joints[joints] = (self.base_joint)
            else:

                self.temp_joints[joints] = ((
                    self.temp_joints[joints-1][0] 
                    + math.cos(self.temp_joint_angles[joints-1])*self.arm_lengths[joints-1], 

                    self.temp_joints[joints-1][1] 
                    - math.sin(self.temp_joint_angles[joints-1])*self.arm_lengths[joints-1]
                ))

        return self.temp_joints

    def Set_Joint_pos(self, joints):
        self.joints = joints

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
        if joint == 0:
            pass
        else:
            self.test_joint_angle = self.joint_angles[joint] - math.radians(angle)
            self.text_relative_angle = (180 - math.degrees(self.joint_angles[joint-1])) + math.degrees(self.test_joint_angle)
            if self.text_relative_angle < 5 or self.text_relative_angle > 355:
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
        self.Set_Joint_pos(
            self.calculate_joint_pos()
        )

    # sets all 3 angles for the arm practically teleporting it 
    def Set_Angles(self, angle_1, angle_2, angle_3):
        new_angles = [angle_1, angle_2, angle_3]

        for i in range(len(self.joint_angles)):
            self.joint_angles[i] = new_angles[i]

            if i == 0:
                self.relative_angles[i] = self.joint_angles[i]
            else:
                self.relative_angles[i] = math.radians((180 - math.degrees(self.joint_angles[i-1])) + math.degrees(self.joint_angles[i]))

        self.Set_Joint_pos(
            self.calculate_joint_pos()
        )

    # calculates all the angles between the start and end position creating a route for the arm
    def Route_Taker(self, new_1, new_2, new_3, display, world, planner):

        new = [new_1, new_2, new_3]
        planner.node_dictionary = {}
        planner.closed_set = set() 

        self.start_tuple = (
            round(norm_deg(self.joint_angles[0]) / RESOLUTION),
            round(norm_deg(self.joint_angles[1]) / RESOLUTION),
            round(norm_deg(self.joint_angles[2]) / RESOLUTION)
        )

        self.end_tuple = (
            round(norm_deg(new_1) / RESOLUTION),
            round(norm_deg(new_2) / RESOLUTION),
            round(norm_deg(new_3) / RESOLUTION)
        )

        print("start:", self.start_tuple, "end:", self.end_tuple)

        self.G_value_dictionary = {}    
        self.came_from = {}

        self.G_value_dictionary = {self.start_tuple: 0}
        open_heap = []
        start_f = node_cost_check(self.start_tuple, self.end_tuple) * WEIGHT 
        heapq.heappush(open_heap, (start_f, self.start_tuple))
        it = 0
        while True:
            print(it)
            it += 1
            if not open_heap:
                        print(f"No route found after {it} tries")
                        return None

            current_f, self.current_node = heapq.heappop(open_heap)

            if self.current_node in planner.closed_set:
                continue

            if self.current_node == self.end_tuple:
                break

            planner.closed_set.add(self.current_node)
            tentative_g = self.G_value_dictionary[self.current_node] + 1
            
            options = node_neighbor(self.current_node, planner, world.robot, world)
            for option in range(len(options)):
                if options[option] not in self.G_value_dictionary or tentative_g < self.G_value_dictionary[options[option]]:
                    self.G_value_dictionary[options[option]] = tentative_g
                    self.came_from[options[option]] = self.current_node
                    new_f = node_cost_check(options[option], self.end_tuple) * WEIGHT + self.G_value_dictionary[options[option]]
                    heapq.heappush(open_heap, (new_f, options[option]))

        path_angles = [self.end_tuple]
        node = self.end_tuple
        while node != self.start_tuple:
            node = self.came_from[node]
            path_angles.append(node)
        path_angles.reverse()
        print("success")
        
        return path_angles