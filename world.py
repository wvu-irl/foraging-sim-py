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
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, v_filepath_list, q_filepath_list, arbitration_type_list, num_time_steps):
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

        # Record the food positions, headings, and clusters lists
        self.food_pos, self.food_heading, self.food_cluster = self.map.findFoodInfo()
        self.num_food = len(self.food_pos)
        self.num_clusters = max(self.food_cluster) + 1
        
        # Set max battery
        self.battery_size = battery_size

        # Set max heading state size
        self.heading_size = heading_size

        # Record policy, V and Q filepaths, if offline policies are to be used
        self.policy_filepath_list = policy_filepath_list
        self.v_filepath_list = v_filepath_list
        self.q_filepath_list = q_filepath_list
        if isinstance(self.policy_filepath_list[0], list):
            self.num_models = len(self.policy_filepath_list[0])
        else:
            self.num_models = 1

        # Record arbitration type, if mm-mdp
        self.arbitration_type_list = arbitration_type_list

        # Record the full state dimensions and number of actions
        self.full_state_dimensions = {"x_size" : self.map_shape[0], "y_size" : self.map_shape[1], "has_food_size" : 2, "battery_size" :self.battery_size, "num_food" : self.num_food, "heading_size" : self.heading_size}
        self.num_full_states =  self.full_state_dimensions["x_size"] * self.full_state_dimensions["y_size"] * self.full_state_dimensions["has_food_size"] * self.full_state_dimensions["battery_size"] * (2 ** self.full_state_dimensions["num_food"]) * self.full_state_dimensions["heading_size"]
        self.num_actions = Actions.DROP + 1

        # Record number of robots and initialize lists of robots, states, and models
        self.num_robots = len(self.robot_personality_list)
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
                    if robot_personality_list[robot_id] in [0, 1, 2, 3, 7, 8, 9, 10, 11, 12]:
                        robot_states = FullStates()
                    elif robot_personality_list[robot_id] in [4, 5, 6]:
                        robot_states = SwarmFullStates()
                    robot_states.x = x
                    robot_states.y = y
                    robot_states.battery = self.battery_size - 1
                    robot_states.food_state = int((2 ** self.num_food) - 1)
                    self.true_robot_states[robot_id] = copy.deepcopy(robot_states)
                    self.true_constants[robot_id] = {"map_shape" : self.map_shape, "battery_size" : self.battery_size, "home_pos" : self.home_pos, "heading_size" : self.heading_size, \
                            "num_food" : self.num_food, "food_pos" : self.food_pos, "num_clusters" : self.num_clusters, "food_cluster" : self.food_cluster, "food_heading" : self.food_heading, \
                             "num_actions" : self.num_actions, "id" : robot_id, "personality" : robot_personality_list[robot_id]}
                    self.robot_constants[robot_id] = {"map_shape" : self.map_shape, "battery_size" : self.battery_size, "home_pos" : self.home_pos, \
                            "num_food" : self.num_food, "food_pos" : self.food_pos, "num_clusters" : self.num_clusters, "food_cluster" : self.food_cluster, "num_actions" : self.num_actions, \
                            "id" : robot_id, "personality" : robot_personality_list[robot_id]}
                    self.results_metrics[robot_id] = ResultsMetrics()
                    if robot_personality_list[robot_id] == 0:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 1:
                        self.robot[robot_id] = randomSelectRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 2:
                        self.robot[robot_id] = randomSelectRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 3:
                        self.robot[robot_id] = randomSelectRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 4:
                        self.robot[robot_id] = randomSelectLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 5:
                        self.robot[robot_id] = randomSelectLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 6:
                        self.robot[robot_id] = randomSelectLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction
                        self.use_submap[robot_id] = True
                    elif robot_personality_list[robot_id] == 7:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : self.policy_filepath_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 8:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : self.policy_filepath_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 9:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : self.policy_filepath_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 10:
                        self.robot[robot_id] = SingleMMMDPRobot({"num_models": self.num_models, "policy_filepath" : self.policy_filepath_list[robot_id], "v_filepath" : self.v_filepath_list[robot_id], "q_filepath" : self.q_filepath_list[robot_id], "arbitration_type" : self.arbitration_type_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.robot[robot_id].T = mdpDirectionalFoodTransitionModel
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 11:
                        self.robot[robot_id] = SingleMMMDPRobot({"num_models": self.num_models, "policy_filepath" : self.policy_filepath_list[robot_id], "v_filepath" : self.v_filepath_list[robot_id], "q_filepath" : self.q_filepath_list[robot_id], "arbitration_type" : self.arbitration_type_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.robot[robot_id].T = mdpDirectionalFoodTransitionModel
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False
                    elif robot_personality_list[robot_id] == 12:
                        self.robot[robot_id] = SingleMMMDPRobot({"num_models": self.num_models, "policy_filepath" : self.policy_filepath_list[robot_id], "v_filepath" : self.v_filepath_list[robot_id], "q_filepath" : self.q_filepath_list[robot_id], "arbitration_type" : self.arbitration_type_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.robot[robot_id].T = mdpDirectionalFoodTransitionModel
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                        self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        self.true_reward_function[robot_id] = mdpRewardFunction 
                        self.use_submap[robot_id] = False

    def updateRobotObservation(self, i):
        #if self.use_submap[i]:
        submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range, self.true_robot_states)
        observation = self.true_observation_model[i](self.true_robot_states[i], submap, self.true_constants[i])
        #else:
        #    observation = self.true_observation_model[i](self.true_robot_states[i], self.true_constants[i])
        self.robot[i].stateEstimator(observation)

    def executeRobotAction(self, i):
        action = self.robot[i].chooseAction()
        # DEPRACATED
        #if self.use_submap[i]:
        #    submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, 1, self.true_robot_states)
        #    (new_states, new_submap) = self.true_transition_model[i](self.true_robot_states[i], submap, action, self.true_constants[i])
        #    self.map.setSubMap(new_states.x, new_states.y, new_submap)
        #else:
        state_outcomes, state_outcome_probs = self.true_transition_model[i](self.true_robot_states[i], action, self.true_constants[i])
        if len(state_outcomes) > 1:
            rng = np.random.default_rng()
            new_states = rng.choice(state_outcomes, p=state_outcome_probs)
        else:
            new_states = state_outcomes[0]
        new_food_map = getFoodMapFromBinary(new_states.food_state, self.num_food, self.food_pos, self.map_shape)
        for j in range(self.num_food):
            if new_food_map[self.food_pos[j][0], self.food_pos[j][1]]:
                new_food_map[self.food_pos[j][0], self.food_pos[j][1]] = self.food_heading[j]
        self.map.map[MapLayer.FOOD, :, :] = new_food_map
        self.map.map[MapLayer.ROBOT, self.true_robot_states[i].x, self.true_robot_states[i].y] = 0
        self.map.map[MapLayer.ROBOT, new_states.x, new_states.y] = i+1 

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

        if state_prime_at_home == True and state_prime.has_food == False and state.has_food == True:
            results_out.num_food_retrieved = results_in.num_food_retrieved + 1

        if state_prime.x != state.x or state_prime.y != state.y:
            results_out.total_distance_traversed = results_in.total_distance_traversed + 1

        if state_prime.battery == 0 and state.battery > 0:
            results_out.battery_died = True

        return results_out
