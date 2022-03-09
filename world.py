import numpy as np
import copy
from foraging_map import ForagingMap
from states import *
from robot import *
from transition_models import *
from observation_models import *
from reward_functions import *
from results_metrics import ResultsMetrics

class World:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, num_time_steps):
        # Initialize map
        self.map = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
        self.map_shape = self.map.map_shape

        # Record perception range
        self.perception_range = perception_range

        # Record robot personality list
        self.robot_personality_list = robot_personality_list

        # Record number of timesteps
        self.num_time_steps = num_time_steps

        # Record home location
        self.home_pos = self.map.findHomePosition(0)

        # Record the food positions and headings lists
        self.food_pos, self.food_heading = self.map.findFoodInfo()
        self.num_food = len(self.food_pos)

        # Set max battery
        self.battery_size = battery_size

        # Record number of robots and initialize lists of robots, states, and models
        self.num_robots = len(robot_personality_list)
        self.robot = [None] * self.num_robots
        self.true_constants = [None] * self.num_robots
        self.robot_constants = [None] * self.num_robots
        self.true_robot_states = [None] * self.num_robots
        self.true_observation_model = [None] * self.num_robots
        self.true_transition_model = [None] * self.num_robots
        self.true_reward_function = [None] * self.num_robots
        self.true_total_reward = np.zeros(self.num_robots)
        self.use_submap = [False] * self.num_robots
        self.results_metrics = [None] * self.num_robots

        # Find position of robots in map and initialize true states
        for x in range(self.map_shape[0]):
            for y in range(self.map_shape[1]):
                robot_id = self.map.map[MapLayer.ROBOT, x, y] - 1
                if robot_id >= 0: # TODO: improve this initialization to initialize different robots differently (i.e., different heading, etc)
                    robot_states = FullStates()
                    robot_states.x = x
                    robot_states.y = y
                    robot_states.battery = self.battery_size - 1
                    print("battery: {0}".format(robot_states.battery))
                    robot_states.food_state = int((2 ** self.num_food) - 1)
                    self.true_robot_states[robot_id] = robot_states
                    self.true_constants[robot_id] = {"map_shape" : self.map_shape, "battery_size" : self.battery_size, "home_pos" : self.home_pos, \
                            "num_food" : self.num_food, "food_pos" : self.food_pos, "food_heading" : self.food_heading, "id" : robot_id, "personality" : robot_personality_list[robot_id]}
                    self.robot_constants[robot_id] = {"map_shape" : self.map_shape, "battery_size" : self.battery_size, "home_pos" : self.home_pos, \
                            "num_food" : self.num_food, "food_pos" : self.food_pos, "id" : robot_id, "personality" : robot_personality_list[robot_id]}
                    self.results_metrics[robot_id] = ResultsMetrics()
                    if robot_personality_list[robot_id] == 0:
                        self.robot[robot_id] = SimpleDeterministicRobot({}, self.robot_constants[robot_id])
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = deterministicTransitionModel
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 1:
                        self.robot[robot_id] = SimpleLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 2:
                        self.robot[robot_id] = SimpleLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 3:
                        self.robot[robot_id] = SimpleLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 4:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 5:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 6:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = directionalFoodTransitionModel1
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 7:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : "policies/vi_policy.npy"}, self.robot_constants[robot_id])
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainMDPObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 8:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : "policies/vi_policy.npy"}, self.robot_constants[robot_id])
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainMDPObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 9:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : "policies/vi_policy.npy"}, self.robot_constants[robot_id])
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainMDPObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False

    def updateRobotObservation(self, i):
        if self.use_submap[i]:
            submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range, self.true_robot_states)
            observation = self.true_observation_model[i](self.true_robot_states[i], submap, self.true_constants[i])
        else:
            observation = self.true_observation_model[i](self.true_robot_states[i], self.true_constants[i])
        self.robot[i].stateEstimator(observation)

    def executeRobotAction(self, i):
        action = self.robot[i].chooseAction()
        if self.use_submap[i]:
            submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, 1, self.true_robot_states)
            (new_states, new_submap) = self.true_transition_model[i](self.true_robot_states[i], submap, action, self.true_constants[i])
            self.map.setSubMap(new_states.x, new_states.y, new_submap)
        else:
            new_states = self.true_transition_model[i](self.true_robot_states[i], action, self.true_constants[i])
        self.true_total_reward[i] += self.true_reward_function[i](self.true_robot_states[i], action, new_states, self.true_constants[i])
        self.results_metrics[i] = self.updateResultsMetrics(self.results_metrics[i], self.true_robot_states[i], new_states, self.true_constants[i])
        self.true_robot_states[i] = new_states

    def simulationStep(self):
        for i in range(self.num_robots):
            self.updateRobotObservation(i)
            self.executeRobotAction(i)

    def updateResultsMetrics(self, results_in, state, state_prime, constants):
        results_out = copy.deepcopy(results_in)
        home_pos = constants["home_pos"]
        if state.x == home_pos[0] and state.y == home_pos[1]:
            state_at_home = True
        else:
            state_at_home = False

        if state_prime.x == home_pos[0] and state_prime.y == home_pos[1]:
            state_prime_at_home = True
        else:
            state_prime_at_home = False

        if state_prime_at_home == True and state_at_home == False:
            results_out.num_times_home_visited = results_in.num_times_home_visited + 1

        if state_prime_at_home == True and state_prime.has_food == False and state.has_food == False:
            results_out.num_food_retrieved = results_in.num_food_retrieved + 1

        if state_prime.x != state.x or state_prime.y != state.y:
            results_out.total_distance_traversed = results_in.total_distance_traversed + 1

        return results_out
