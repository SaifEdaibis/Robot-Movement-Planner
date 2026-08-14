import math
import pygame
import heapq

from robot import norm_deg
import settings

#--- Path Planner -----------------------------------------------------------------------------------------------------------------------------------------

# This class detrmines the angles necescary to go from the start to end positions
class Path_Planner:
    def __init__(self):
        self.node_dictionary = {}
        self.closed_set = set()

    def final_angles(self, end_pos, robot, world, elbow_sign):

        for tries in range(1,65):
            print(tries)
            # stands for hypotenuse, this is the distance between the base and the 3rd joint. This cannot exceed 200 or the program will crash
            hypo = 250
            # how many times my program has tried to get a sutable angle
            theta_3_atempts = 0

            
            #the target position
            final_x, final_y = end_pos
            start_x, start_y = settings.BASE_JOINT

            # the angle for the third joint
            theta_3 = math.pi *((1/32) * tries)

            #the positions of the third joint
            final_x = final_x - math.cos(theta_3) * robot.arm_lengths[-1]
            final_y = final_y + math.sin(theta_3) * robot.arm_lengths[-1]

            #the differinces between the base joint and third joint. CANNOT EXCEED 200
            delta_x = final_x - start_x
            delta_y = final_y - start_y

            hypo = math.sqrt((delta_x ** 2) + (delta_y ** 2))
            theta_3_atempts += 1

            if hypo >= 200:
                continue
            
            #the angle between the first and second arm which is the relative_angle for the second joint
            try:
                inside_angle_b = elbow_sign * math.acos(
                                    ((delta_x ** 2) 
                                    + (delta_y ** 2)
                                    - (robot.arm_lengths[0] ** 2)
                                    - (robot.arm_lengths[1] ** 2))
                                    / (-2 * robot.arm_lengths[0] * robot.arm_lengths[1])
                            )
            except:
              print(f"tries={tries}: hypo={hypo:.2f} - Cannot reach position (arm too short)")
              continue

            # relative angle for the sceond joint
            relative_angle_2 = inside_angle_b

            # the angle between the first arm and the hypotnuse created from the base joint to the third joint
            try:
                inside_angle_a = elbow_sign * math.acos(
                                            ( (robot.arm_lengths[1] ** 2)
                                            - (delta_x ** 2) 
                                            - (delta_y ** 2)
                                            - (robot.arm_lengths[0] ** 2))
                                            / (-2 * robot.arm_lengths[0] * math.sqrt((delta_x **2)+ (delta_y ** 2)))
                                    )
            except:
                print(f"tries={tries}: hypo={hypo:.2f} - Cannot reach position (arm too short)")
                continue

            # the angle between the hypotnuse created from the base joint to the third joint and the x axis
            base_angle_a = math.atan2(-delta_y,delta_x)

            # the joint angles for the first two joints
            joint_angle_1 = inside_angle_a + base_angle_a
            joint_angle_2 = relative_angle_2 - math.radians(180) + joint_angle_1

            tuple = (
                        round(norm_deg(joint_angle_1) / settings.RESOLUTION ),
                        round(norm_deg(joint_angle_2) / settings.RESOLUTION ),
                        round(norm_deg(theta_3) / settings.RESOLUTION )
                    )

            if self.position_validifier(tuple, robot, world):
                print("success")
                return (joint_angle_1, joint_angle_2, theta_3)

        return None

    def position_validifier(self, node_tuple, robot, world):
        i, j, k = node_tuple

        if (i, j, k) in self.node_dictionary:
            return self.node_dictionary[(i, j, k)]
        
        angle_1 = math.radians(i * settings.RESOLUTION )
        angle_2 = math.radians(j * settings.RESOLUTION )
        angle_3 = math.radians(k * settings.RESOLUTION )
        self.test_joint_pos = robot.calculate_joint_pos(angle_1, angle_2, angle_3)

        for obstacle in world.obstacles:

            for joint_pos in self.test_joint_pos:
                padded_rect = obstacle.rect.inflate(settings.JOINT_COLLISION_PADDING, settings.JOINT_COLLISION_PADDING)
                if padded_rect.collidepoint(joint_pos):
                    self.node_dictionary[(i, j, k)] = False
                    return False

            for arm in range(len(self.test_joint_pos) - 1):
                padded_rect = obstacle.rect.inflate(settings.COLLISION_PADDING, settings.COLLISION_PADDING)
                self.collision = bool(padded_rect.clipline(self.test_joint_pos[arm], self.test_joint_pos[arm + 1]))

                if self.collision:
                    self.node_dictionary[(i, j, k)] = False
                    return False

        self.node_dictionary[(i, j, k)] = True
        return True