import math
import pygame
import heapq

import settings

#--- Helper Functions -------------------------------------------------------------------------------------------------------------------------------------------------------------------

def relative_angles_from_tuple(node_tuple):
    i, j, k = [x * settings.RESOLUTION  for x in node_tuple]
    rel_2 = (180 - i) + j
    rel_3 = (180 - j) + k
    return rel_2 % 360, rel_3 % 360

def circular_diff(a, b, steps=settings.STEPS_PER_REV):
    diff = abs(a - b) % steps
    return min(diff, steps - diff)

# converts degrees into node values
def norm_deg(rad):
    return round(math.degrees(rad) % 360)

# checks the distance between the node and the final node to help determine the cost
def node_cost_check(node, final_node):
    d0 = circular_diff(node[0], final_node[0])
    d1 = circular_diff(node[1], final_node[1])
    d2 = circular_diff(node[2], final_node[2])
    return math.sqrt(d0**2 + d1**2 + d2**2)

# determines the neighboring nodes and sets them as possible options
def node_neighbor(node_tuple, planner, robot, world):
    node_options_list = []

    key = {
        0: [1, -1, 0, 0, 0, 0],
        1: [0, 0, 1, -1, 0, 0],
        2: [0, 0, 0, 0, 1, -1],
    }

    for i in range(6):
            test_tuple = (
                (node_tuple[0] + key[0][i]) % settings.STEPS_PER_REV, 
                (node_tuple[1] + key[1][i]) % settings.STEPS_PER_REV,
                (node_tuple[2] + key[2][i]) % settings.STEPS_PER_REV
            )

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

        self.joints = [None] * settings.JOINT_NUMBERS
        self.base_joint =   settings.BASE_JOINT

        # creates a list that records the angle of each joint
        self.joint_angles = [
            settings.BASE_JOINT_ANGLE ,
            settings.SECOND_JOINT_ANGLE,
            settings.THIRD_JOINT_ANGLE,
        ]

        # creates a list that records the angle of each arm realtive to the prevous arm. This is used to make arms position fixed
        self.relative_angles = [
            settings.BASE_JOINT_ANGLE ,

            math.radians((180 - math.degrees(settings.BASE_JOINT_ANGLE)) 
            + math.degrees(settings.SECOND_JOINT_ANGLE)),

            math.radians((180 - math.degrees(settings.SECOND_JOINT_ANGLE))
            + math.degrees(settings.THIRD_JOINT_ANGLE))
        ]

        # creates list that records starting arm lengths
        self.arm_lengths = [
            settings.STARTING_ARM_LENGTH,
            settings.STARTING_ARM_LENGTH,
            settings.STARTING_ARM_LENGTH
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
                settings.ARM_COLOR,
                self.joints[arm],
                self.joints[arm+1], 
                settings.ARM_WIDTH
            )

        for joint in range(len(self.joints)):

            #draws the robots outer joints
            pygame.draw.circle(
                screen,
                settings.JOINT_COLOR,
                self.joints[joint],
                settings.JOINT_RADIUS,
                width = 0
            )

            #draws the robots inner joints
            pygame.draw.circle(
                screen,
                settings.ARM_COLOR,
                self.joints[joint],
                settings.INNER_JOINT_RADIUS,
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
    def Route_Taker(self, start_1, start_2, start_3, new_1, new_2, new_3, display, world, planner):

        new = [new_1, new_2, new_3]
        planner.node_dictionary = {}
        planner.closed_set = set() 

        self.start_tuple = (
            round(norm_deg(start_1) / settings.RESOLUTION ) % settings.STEPS_PER_REV,
            round(norm_deg(start_2) / settings.RESOLUTION ) % settings.STEPS_PER_REV,
            round(norm_deg(start_3) / settings.RESOLUTION ) % settings.STEPS_PER_REV
        )

        self.end_tuple = (
            round(norm_deg(new_1) / settings.RESOLUTION ) % settings.STEPS_PER_REV,
            round(norm_deg(new_2) / settings.RESOLUTION ) % settings.STEPS_PER_REV,
            round(norm_deg(new_3) / settings.RESOLUTION ) % settings.STEPS_PER_REV
        )

        print("start:", self.start_tuple, "end:", self.end_tuple)

        self.G_value_dictionary = {}    
        self.came_from = {}

        self.G_value_dictionary = {self.start_tuple: 0}
        open_heap = []
        start_f = node_cost_check(self.start_tuple, self.end_tuple) * settings.WEIGHT 
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
                    new_f = node_cost_check(options[option], self.end_tuple) * settings.WEIGHT + self.G_value_dictionary[options[option]]
                    heapq.heappush(open_heap, (new_f, options[option]))

        path_angles = [self.end_tuple]
        node = self.end_tuple
        while node != self.start_tuple:
            node = self.came_from[node]
            path_angles.append(node)
            
        path_angles.reverse()
        print("path found!")
        
        return path_angles