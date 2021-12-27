import numpy as np
from foraging_map import ForagingMap
from states import *
from robot import *
from transition_models import *
from observation_models import *

class World:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range):
        # Initialize map
        self.map = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
        self.map_shape = self.map.map_shape

        # Record perception range
        self.perception_range = perception_range

        # Initialize robots, true perception models, and true transition models
        self.num_robots = len(robot_personality_list)
        self.robot = []
        self.true_robot_states = []
        self.true_observation_model = []
        self.true_transition_model = []
        for i in range(self.num_robots):
            if robot_personality_list[i] == 0:
                self.robot.append(SimpleDeterministicRobot({}))
                initial_states = States
                #TODO set initial_states, some parts based on map
                self.true_robot_states.append(initial_states)
                self.true_observation_model.append(fullyAccurateAndCertainObservationModel)
                self.true_transition_model.append(deterministicTransitionModel)
            elif robot_personality_list[i] == 1:
                # self.robot.append(RobotType1(...))
                # self.true_robot_states.append(initial_states)
                # self.true_perception_model.append(observationType1)
                # self.true_transition_model.append(transitionType1)

        # Record constants known to the true simulation
        self.true_constants = {"map_shape" : self.map_shape}

        # Record constants known to the robots (possibly different than the true simulation)
        self.robot_constants = {"map_shape" : self.map_shape}

    def simulationStep(self):
        for i in range(self.num_robots):
            updateRobotObservation(i)
            executeRobotAction(i)

    def updateRobotObservation(self, i):
        submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range, self.true_robot_states)
        observation = self.true_observation_model[i](self.true_robot_states[i], submap, self.true_constants)
        self.robot[i].stateEstimator(observation)

    def executeRobotAction(self, i):
        self.robot[i].chooseAction()
        action = self.robot[i].next_action
        submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, 1, self.true_robot_states)
        (new_states, new_submap) = self.true_transition_model[i](self.true_robot_states[i], submap, action, self.true_constants)
        self.true_robot_states[i] = new_states
        self.map.setSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, new_submap)
