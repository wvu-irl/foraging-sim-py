import numpy as np
from foraging_map import ForagingMap
from states import States

class World:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range):
        # Initialize map
        self.map = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
        # TODO: need to maintain robot true states separately from self.robot, below. self.robot is meant to encapsulate what the robot knows. In order to be generalizable, these states need to have the ability to be inaccurate or uncertain. Need to store true states separately, at this level, in World

        # Record perception range
        self.perception_range = perception_range

        # Initialize robots, true perception models, and true transition models
        self.num_robots = len(robot_personality_list)
        self.robot = []
        self.true_robot_states = []
        self.true_perception_model = []
        self.true_transition_model = []
        for i in range(self.num_robots):
            if robot_personality_list[i] == 0:
                # self.robot.append(RobotType0(...))
                # self.true_robot_states.append(initial_states)
                # self.true_perception_model.append(perceptionType0)
                # self.true_transition_model.append(transitionType0)
            elif robot_personality_list[i] == 1:
                # self.robot.append(RobotType1(...))
                # self.true_robot_states.append(initial_states)
                # self.true_perception_model.append(perceptionType1)
                # self.true_transition_model.append(transitionType1)

    def executeRobotActions(self):
        for i in range(self.num_robots):
            action = self.robot[i].next_action
            # ...

    def updateRobotStatesAndLocalMaps(self):
        for i in range(self.num_robots):
            (food_submap, home_submap, obstacle_submap, robot_submap) = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range)
            (robot_perceived_states, perceived_food_submap,
