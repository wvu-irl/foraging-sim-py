import numpy as np
from foraging_map import ForagingMap
from states import *
from robot import *
from transition_models import *
from observation_models import *

class World:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, num_time_steps):
        # Initialize map
        self.map = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
        self.map_shape = self.map.map_shape

        # Record perception range
        self.perception_range = perception_range

        # Record robot personality list
        self.robot_personality_list = robot_personality_list

        # Record number of timesteps
        self.num_time_steps = num_time_steps

        # Record home location TODO: change this to allow for different homes based on different robot IDs or not
        self.home_pos = self.map.findHomePosition(0)
        
        # Record constants known to the true simulation
        self.true_constants = {"map_shape" : self.map_shape, "home_pos" : self.home_pos}

        # Record constants known to the robots (possibly different than the true simulation)
        self.robot_constants = {"map_shape" : self.map_shape, "home_pos" : self.home_pos}

        # Record number of robots and initialize lists of robots, states, and models
        self.num_robots = len(robot_personality_list)
        self.robot = [None] * self.num_robots
        self.true_robot_states = [None] * self.num_robots
        self.true_observation_model = [None] * self.num_robots
        self.true_transition_model = [None] * self.num_robots

        # Find position of robots in map and initialize true states
        for x in range(self.map_shape[0]):
            for y in range(self.map_shape[1]):
                robot_id = self.map.map[MapLayer.ROBOT, x, y] - 1
                if robot_id >= 0: # TODO: improve this initialization to initialize different robots differently (i.e., different heading, etc)
                    robot_states = FullStates()
                    robot_states.x = x
                    robot_states.y = y
                    robot_states.battery = 100.0
                    robot_states.personality = robot_personality_list[robot_id]
                    robot_states.robot_id = robot_id
                    self.true_robot_states[robot_id] = robot_states
                    if robot_personality_list[robot_id] == 0:
                        self.robot[robot_id] = SimpleDeterministicRobot({}, self.robot_constants)
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = deterministicTransitionModel
                    elif robot_personality_list[robot_id] == 1:
                        self.robot[robot_id] = SimpleLocalInteractionRandomGrabRobot({}, self.robot_constants)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                    elif robot_personality_list[robot_id] == 2:
                        self.robot[robot_id] = SimpleLocalInteractionRandomGrabRobot({}, self.robot_constants)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                    elif robot_personality_list[robot_id] == 3:
                        self.robot[robot_id] = SimpleLocalInteractionRandomGrabRobot({}, self.robot_constants)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                    elif robot_personality_list[robot_id] == 4:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                    elif robot_personality_list[robot_id] == 5:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                    elif robot_personality_list[robot_id] == 6:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1

    def updateRobotObservation(self, i):
        submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range, self.true_robot_states)
        observation = self.true_observation_model[i](self.true_robot_states[i], submap, self.true_constants)
        self.robot[i].stateEstimator(observation)

    def executeRobotAction(self, i):
        action = self.robot[i].chooseAction()
        submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, 1, self.true_robot_states)
        (new_states, new_submap) = self.true_transition_model[i](self.true_robot_states[i], submap, action, self.true_constants)
        self.true_robot_states[i] = new_states
        self.map.setSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, new_submap)

    def simulationStep(self):
        for i in range(self.num_robots):
            self.updateRobotObservation(i)
            self.executeRobotAction(i)
