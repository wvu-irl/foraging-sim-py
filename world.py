import numpy as np
import copy
from foraging_map import ForagingMap
from states import *
from robot import *
from transition_models import *
from observation_models import *
from reward_functions import *
from results_metrics import ResultsMetrics
import matplotlib.pyplot as plt
from map_viz import displayMap
from prev_exp import PrevExpData
import rospy # TODO: this is a bad hack. Needs removed so sims do not depend on ROS

class World:
    def __init__(self, trial_num, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, v_filepath_list, q_filepath_list, arbitration_type_list, use_prev_exp, prev_exp_filepath, num_time_steps, heading_change_times, food_respawn, real_world_exp = False, manual_control = False):
        # If real world experiment, import air hockey interface (if not, don't import so not dependent on ROS)
        if real_world_exp:
            from air_hockey_interface import AirHockeyInterface

        # Record trial number (needed for some data saving purposes)
        self.trial_num = trial_num

        # Record if manual control is enabled (for debugging)
        self.manual_control = manual_control

        # Initialize map
        self.map = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
        self.map_shape = self.map.map_shape

        # Record perception range
        self.perception_range = perception_range

        # Record robot personality list
        self.robot_personality_list = robot_personality_list

        # Record number of timesteps
        self.num_time_steps = num_time_steps

        # Record heading change times
        self.heading_change_times = heading_change_times

        # Record if food should respawn
        self.food_respawn = food_respawn

        # Record home location
        self.home_pos , self.home_region = self.map.findHome()

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
        if self.policy_filepath_list and isinstance(self.policy_filepath_list[0], list):
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
        self.real_world_interface = [None] * self.num_robots
        self.true_reward_function = [None] * self.num_robots
        self.true_total_reward = np.zeros(self.num_robots)
        self.use_full_map = [False] * self.num_robots
        self.results_metrics = [None] * self.num_robots
        self.total_food_retrieved = 0

        # Record whether previous experience is to be used and the filepath if so
        self.use_prev_exp = use_prev_exp
        self.prev_exp_filepath = prev_exp_filepath

        # If previous experience is to be used, load the previous experience data and initialize other data containers
        if self.use_prev_exp:
            self.prev_exp_data = PrevExpData()
            self.prev_exp_data.load(self.prev_exp_filepath)        
            self.num_prev_exp_robots = self.prev_exp_data.last_trial_written[0] + 1
            self.prev_exp_robot_id = [self.prev_exp_data.robot_id[0] + i + self.num_robots for i in range(self.num_prev_exp_robots)]
            self.prev_exp_states = [UnknownMapFullStates() for i in range(self.num_prev_exp_robots)]
            self.prev_exp_constants = [{"id" : self.prev_exp_robot_id[i], "personality" : self.prev_exp_data.personality[0], "phantom" : True} for i in range(self.num_prev_exp_robots)]

        # Record if this is a real world experiment (True) or simulation (False)
        self.real_world_exp = real_world_exp

        # Find position of robots in map and initialize true states
        for x in range(self.map_shape[0]):
            for y in range(self.map_shape[1]):
                robot_id = self.map.map[MapLayer.ROBOT, x, y] - 1
                if robot_id >= 0: # TODO: improve this initialization to initialize different robots differently (i.e., different heading, etc)
                    print("Initialization: robot x,y = [{0},{1}]".format(x, y))
                    if robot_personality_list[robot_id] in [0, 1, 2, 3, 7, 8, 9, 10, 11, 12]:
                        robot_states = FullStates()
                    elif robot_personality_list[robot_id] in [4, 5, 6]:
                        robot_states = SwarmFullStates()
                    elif robot_personality_list[robot_id] in [13, 14, 15, 16, 17, 18]:
                        robot_states = UnknownMapFullStates()
                    else:
                        raise RuntimeError("robot personality: ({0}) not in list of valid personalities".format(robot_personality_list[robot_id]))
                    robot_states.x = x
                    robot_states.y = y
                    robot_states.battery = self.battery_size - 1
                    robot_states.food_state = int((2 ** self.num_food) - 1)
                    self.true_robot_states[robot_id] = copy.deepcopy(robot_states)
                    self.true_constants[robot_id] = {"map_shape" : self.map_shape, "battery_size" : self.battery_size, "home_pos" : self.home_pos, "home_region" : self.home_region, "heading_size" : self.heading_size, "num_food" : self.num_food, "food_pos" : self.food_pos, "num_clusters" : self.num_clusters, "food_cluster" : self.food_cluster, "food_heading" : self.food_heading, "num_actions" : self.num_actions, "id" : robot_id, "personality" : robot_personality_list[robot_id], "phantom" : False, "init_pos": (x, y), "perception_range" : self.perception_range}
                    self.robot_constants[robot_id] = {"map_shape" : self.map_shape, "battery_size" : self.battery_size, "home_pos" : self.home_pos, "home_region" : self.home_region, "num_food" : self.num_food, "food_pos" : self.food_pos, "num_clusters" : self.num_clusters, "food_cluster" : self.food_cluster, "num_actions" : self.num_actions, "id" : robot_id, "personality" : robot_personality_list[robot_id], "phantom" : False, "init_pos": (x, y), "perception_range" : self.perception_range}
                    self.results_metrics[robot_id] = ResultsMetrics()
                    self.true_observation_model[robot_id] = fullyAccurateAndCertainObservationModel
                    self.true_reward_function[robot_id] = mdpRewardFunction
                    if robot_personality_list[robot_id] == 0:
                        self.robot[robot_id] = SimpleRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 1:
                        self.robot[robot_id] = randomSelectRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 2:
                        self.robot[robot_id] = randomSelectRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 3:
                        self.robot[robot_id] = randomSelectRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 4:
                        self.robot[robot_id] = randomSelectLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 5:
                        self.robot[robot_id] = randomSelectLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 6:
                        self.robot[robot_id] = randomSelectLocalInteractionRandomGrabRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 7:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : self.policy_filepath_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.use_full_map[robot_id] = True
                    elif robot_personality_list[robot_id] == 8:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : self.policy_filepath_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.use_full_map[robot_id] = True
                    elif robot_personality_list[robot_id] == 9:
                        self.robot[robot_id] = SingleMDPRobot({"policy_filepath" : self.policy_filepath_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.use_full_map[robot_id] = True
                    elif robot_personality_list[robot_id] == 10:
                        self.robot[robot_id] = SingleMMMDPRobot({"num_models": self.num_models, "policy_filepath" : self.policy_filepath_list[robot_id], "v_filepath" : self.v_filepath_list[robot_id], "q_filepath" : self.q_filepath_list[robot_id], "arbitration_type" : self.arbitration_type_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.robot[robot_id].T = mdpDirectionalFoodTransitionModel
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.use_full_map[robot_id] = True
                    elif robot_personality_list[robot_id] == 11:
                        self.robot[robot_id] = SingleMMMDPRobot({"num_models": self.num_models, "policy_filepath" : self.policy_filepath_list[robot_id], "v_filepath" : self.v_filepath_list[robot_id], "q_filepath" : self.q_filepath_list[robot_id], "arbitration_type" : self.arbitration_type_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.robot[robot_id].T = mdpDirectionalFoodTransitionModel
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.use_full_map[robot_id] = True
                    elif robot_personality_list[robot_id] == 12:
                        self.robot[robot_id] = SingleMMMDPRobot({"num_models": self.num_models, "policy_filepath" : self.policy_filepath_list[robot_id], "v_filepath" : self.v_filepath_list[robot_id], "q_filepath" : self.q_filepath_list[robot_id], "arbitration_type" : self.arbitration_type_list[robot_id]}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.robot[robot_id].T = mdpDirectionalFoodTransitionModel
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.use_full_map[robot_id] = True
                    elif robot_personality_list[robot_id] == 13:
                        self.robot[robot_id] = UnknownMapFSMRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 14:
                        self.robot[robot_id] = UnknownMapFSMRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 15:
                        self.robot[robot_id] = UnknownMapFSMRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 16:
                        self.robot[robot_id] = UnknownMapFSMLocalInteractionRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 1
                        self.robot[robot_id].states.heading = 1
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 17:
                        self.robot[robot_id] = UnknownMapFSMLocalInteractionRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 3
                        self.robot[robot_id].states.heading = 3
                        self.use_full_map[robot_id] = False
                    elif robot_personality_list[robot_id] == 18:
                        self.robot[robot_id] = UnknownMapFSMLocalInteractionRobot({}, self.robot_constants[robot_id])
                        self.robot[robot_id].states = copy.deepcopy(robot_states)
                        self.true_robot_states[robot_id].heading = 5
                        self.robot[robot_id].states.heading = 5
                        self.use_full_map[robot_id] = False
                    if self.real_world_exp:
                        self.real_world_interface[robot_id] = AirHockeyInterface(robot_id, self.num_food)
                    else:
                        if self.use_full_map[robot_id]:
                            self.true_transition_model[robot_id] = mdpDirectionalFoodTransitionModelTrue
                        else:
                            self.true_transition_model[robot_id] = unknownMapDirectionalFoodTransitionModelTrue
        # Initialize combined states and constants for first time step, if using previous experience
        if self.use_prev_exp:
            for j in range(self.num_prev_exp_robots):
                self.map.map[MapLayer.ROBOT, self.prev_exp_data.x[j, 0, 0], self.prev_exp_data.y[j, 0, 0]] = self.prev_exp_robot_id[j] + 1
                self.prev_exp_states[j].x = self.prev_exp_data.x[j, 0, 0]
                self.prev_exp_states[j].y = self.prev_exp_data.y[j, 0, 0]
                self.prev_exp_states[j].has_food = self.prev_exp_data.has_food[j, 0, 0]
                self.prev_exp_states[j].battery = self.prev_exp_data.battery[j, 0, 0]
                self.prev_exp_states[j].last_successful_food_x = self.prev_exp_data.last_successful_food_x[j, 0, 0]
                self.prev_exp_states[j].last_successful_food_y = self.prev_exp_data.last_successful_food_y[j, 0, 0]
                self.prev_exp_states[j].last_failed_food_x = self.prev_exp_data.last_failed_food_x[j, 0, 0]
                self.prev_exp_states[j].last_failed_food_y = self.prev_exp_data.last_failed_food_y[j, 0, 0]
                self.prev_exp_states[j].last_approach_dir = self.prev_exp_data.last_approach_dir[j, 0, 0]
            self.combined_states = self.true_robot_states + self.prev_exp_states
            self.combined_constants = self.true_constants + self.prev_exp_constants

    def updateRobotObservation(self, i, t):
        if self.use_full_map[i]:
            food_state = getBinaryFromFoodMap(self.map.map[MapLayer.FOOD, : ,:], self.num_food, self.food_pos)
            self.true_robot_states[i].food_state = food_state
        if self.real_world_exp: 
            self.real_world_interface[i].resetVisibleFood()
            rospy.sleep(0.5)
            self.map.map[MapLayer.FOOD] = np.zeros_like(self.map.map[MapLayer.FOOD])
            for k in range(self.num_food):
                if self.real_world_interface[i].food_visible[k]:
                    food_x = self.real_world_interface[i].visible_food_pos_x[k]
                    food_y = self.real_world_interface[i].visible_food_pos_y[k]
                    if self.map.map[MapLayer.HOME, food_x, food_y] == 0:
                        self.map.map[MapLayer.FOOD, food_x, food_y] = 1
        if self.use_prev_exp:
            for j in range(self.num_prev_exp_robots):
                if t == 0:
                    self.map.map[MapLayer.ROBOT, self.prev_exp_data.x[j, 0, t], self.prev_exp_data.y[j, 0, t]] = self.prev_exp_robot_id[j] + 1
                else:
                    self.map.map[MapLayer.ROBOT, self.prev_exp_states[j].x, self.prev_exp_states[j].y] = 0
                    self.map.map[MapLayer.ROBOT, self.prev_exp_data.x[j, 0, t], self.prev_exp_data.y[j, 0, t]] = self.prev_exp_robot_id[j] + 1
                self.prev_exp_states[j].x = self.prev_exp_data.x[j, 0, t]
                self.prev_exp_states[j].y = self.prev_exp_data.y[j, 0, t]
                self.prev_exp_states[j].has_food = self.prev_exp_data.has_food[j, 0, t]
                self.prev_exp_states[j].battery = self.prev_exp_data.battery[j, 0, t]
                self.prev_exp_states[j].last_successful_food_x = self.prev_exp_data.last_successful_food_x[j, 0, t]
                self.prev_exp_states[j].last_successful_food_y = self.prev_exp_data.last_successful_food_y[j, 0, t]
                self.prev_exp_states[j].last_failed_food_x = self.prev_exp_data.last_failed_food_x[j, 0, t]
                self.prev_exp_states[j].last_failed_food_y = self.prev_exp_data.last_failed_food_y[j, 0, t]
                self.prev_exp_states[j].last_approach_dir = self.prev_exp_data.last_approach_dir[j, 0, t]
            self.combined_states = self.true_robot_states + self.prev_exp_states
            self.combined_constants = self.true_constants + self.prev_exp_constants
            submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range, self.combined_states, self.combined_constants)
        else:
            submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, self.perception_range, self.true_robot_states, self.true_constants)
        #print("Observation submap:")
        #for j in range(len(submap[0])):
        #    if submap[0][j] == MapLayer.FOOD:
        #        print("FOOD: delta x,y = [{0},{1}]".format(submap[1][j]["delta_x"], submap[1][j]["delta_y"]))
        #    elif submap[0][j] == MapLayer.ROBOT:
        #        print("ROBOT: delta x,y = [{0},{1}]".format(submap[1][j]["delta_x"], submap[1][j]["delta_y"]))
        #    elif submap[0][j] == MapLayer.HOME:
        #        print("HOME: delta x,y = [{0},{1}]".format(submap[1][j]["delta_x"], submap[1][j]["delta_y"]))
        #    elif submap[0][j] == MapLayer.OBSTACLE:
        #        print("OBSTACLE: delta x,y = [{0},{1}]".format(submap[1][j]["delta_x"], submap[1][j]["delta_y"]))
        observation = self.true_observation_model[i](self.true_robot_states[i], submap, self.true_constants[i])
        #else:
        #    observation = self.true_observation_model[i](self.true_robot_states[i], self.true_constants[i])
        self.robot[i].stateEstimator(observation)

    def executeRobotAction(self, i):
        if self.manual_control:
            user_input = input("")
            if user_input == '6':
                action = Actions.MOVE_E
            elif user_input == '9':
                action = Actions.MOVE_NE
            elif user_input == '8':
                action = Actions.MOVE_N
            elif user_input == '7':
                action = Actions.MOVE_NW
            elif user_input == '4':
                action = Actions.MOVE_W
            elif user_input == '1':
                action = Actions.MOVE_SW
            elif user_input == '2':
                action = Actions.MOVE_S
            elif user_input == '3':
                action = Actions.MOVE_SE
            elif user_input == '+':
                action = Actions.GRAB
            elif user_input == '-':
                action = Actions.DROP
            else:
                action = Actions.STAY
        else:
            action = self.robot[i].chooseAction()
        if self.use_full_map[i]:
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
        else:
            if self.use_prev_exp:
                submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, 2, self.combined_states, self.combined_constants)
            else:
                submap = self.map.getSubMap(self.true_robot_states[i].x, self.true_robot_states[i].y, 2, self.true_robot_states, self.true_constants) # TODO: may need to change getSubMap distance to something other than 2 when code is updated to use grids smaller than robot size
            if self.real_world_exp:
                (new_states, new_submap) = self.real_world_interface[i].executeTransition(self.true_robot_states[i], submap, action, self.true_constants[i])
            else:
                (new_states, new_submap) = self.true_transition_model[i](self.true_robot_states[i], submap, action, self.true_constants[i])
            # TODO: temporary solution for passing internal robot data out to the true states for local interacion data communication
            new_states.last_successful_food_x = self.robot[i].states.last_successful_food_x
            new_states.last_successful_food_y = self.robot[i].states.last_successful_food_y
            new_states.last_failed_food_x = self.robot[i].states.last_failed_food_x
            new_states.last_failed_food_y = self.robot[i].states.last_failed_food_y
            new_states.last_approach_dir = self.robot[i].states.last_approach_dir
            # -----------------------------------------------------------------------------------------------------------------------
            self.map.setSubMap(new_states.x, new_states.y, new_submap)

        self.true_total_reward[i] += self.true_reward_function[i](self.true_robot_states[i], action, new_states, self.true_constants[i])
        self.results_metrics[i] = self.updateResultsMetrics(self.results_metrics[i], self.true_robot_states[i], new_states, self.true_constants[i])
        self.true_robot_states[i] = new_states

    def foodRespawnUpdate(self):
        rng = np.random.default_rng()
        # Loop over all food spawning positions in the map
        for i in range(self.num_food):
            # If there is not food at a location currently, perform sample random outcome to respawn
            pos = self.food_pos[i]
            if self.map.map[MapLayer.FOOD, pos[0], pos[1]] == 0:
                if rng.uniform() < 0.01: # TODO: decide if this number needs to be conditional on other factors, like num_robots, num_time_steps, etc
                    self.map.map[MapLayer.FOOD, pos[0], pos[1]] = self.food_heading[i]

    def simulationStep(self, t):
        for i in range(self.num_robots):
            if t in self.heading_change_times:
                self.true_robot_states[i].heading += 2
                self.robot[i].states.heading += 2
                if self.true_robot_states[i].heading > 8:
                    self.true_robot_states[i].heading -= 8
                if self.robot[i].states.heading > 8:
                    self.robot[i].states.heading -= 8
            self.updateRobotObservation(i, t)
            self.executeRobotAction(i)
            if not self.food_respawn:
                if self.total_food_retrieved >= self.num_food: # Terminal condition: all food retrieved
                    return True
        if self.food_respawn:
            self.foodRespawnUpdate()
        return False

    def updateResultsMetrics(self, results_in, state, state_prime, constants):
        results_out = copy.deepcopy(results_in)
        home_region = constants["home_region"]
        if state.x in home_region[0] and state.y in home_region[1]:
            state_at_home = True
        else:
            state_at_home = False

        if state_prime.x in home_region[0] and state_prime.y in home_region[1]:
            state_prime_at_home = True
        else:
            state_prime_at_home = False

        if state_prime_at_home == True and state_at_home == False:
            results_out.num_times_home_visited = results_in.num_times_home_visited + 1

        if state_prime_at_home == True and state_prime.has_food == False and state.has_food == True:
            results_out.num_food_retrieved = results_in.num_food_retrieved + 1
            self.total_food_retrieved += 1

        if state_prime.x != state.x or state_prime.y != state.y:
            results_out.total_distance_traversed = results_in.total_distance_traversed + 1

        if state_prime.battery == 0 and state.battery > 0:
            results_out.battery_died = True

        return results_out
