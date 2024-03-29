import config
from enum import IntEnum, unique
from actions import Actions
from submap_utils import *
from transition_models import *
import numpy as np
from debug_print import debugPrint
from scipy.stats import chi2
from scipy.stats import norm as norm_dist
import matplotlib.pyplot as plt

if False:
    search_goal_fig, search_goal_ax = plt.subplots()
    grab_prob_fig, grab_prob_ax = plt.subplots()
    temp_arr = np.array([[0.0, 0.04], [0.04, 0.0]])
    c_map = search_goal_ax.imshow(temp_arr)
    search_goal_fig.colorbar(c_map)
    c_map = grab_prob_ax.imshow(temp_arr)
    grab_prob_fig.colorbar(c_map)
    search_goal_fig.tight_layout()
    grab_prob_fig.tight_layout()
    plt.ion()

@unique
class FSMState(IntEnum):
    SELECT_TARGET = 0
    APPROACH = 1
    GO_HOME = 2
    CONFIRM_COLLECT = 3
    GO_TO_INIT = 4
    SEARCH = 5

class MovePMFs:
    uniform = np.ones(8) / 8.0
    wide_E = np.array([1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/3.0])
    wide_NE = np.array([1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000])
    wide_N = np.array([0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000, 0.000])
    wide_NW = np.array([0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000])
    wide_W = np.array([0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000])
    wide_SW = np.array([0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000])
    wide_S = np.array([0.000, 0.000, 0.0000, 0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0])
    wide_SE = np.array([1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0])
    narrow_E = np.array([2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/4.0])
    narrow_NE = np.array([1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000, 0.000, 0.000])
    narrow_N = np.array([0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000, 0.000])
    narrow_NW = np.array([0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000])
    narrow_W = np.array([0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000])
    narrow_SW = np.array([0.000, 0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000])
    narrow_S = np.array([0.000, 0.000, 0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0])
    narrow_SE = np.array([1.0/4.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0])

def simpleFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = self.constants["battery_size"] // 2
    map_shape = self.constants["map_shape"]
    home_pos = self.constants["home_pos"]
    home_region = constants["home_region"]
    num_food = self.constants["num_food"]
    food_pos = self.constants["food_pos"]

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SELECT_TARGET:
            debugPrint("fsm_state: SELECT_TARGET")
            food_map = getFoodMapFromBinary(self.states.food_state, num_food, food_pos, map_shape)
            min_distance = float('inf')
            min_distance_index = -1
            for i in range(self.constants["num_food"]):
                if food_map[food_pos[i][0], food_pos[i][1]]:
                    distance = max(abs(self.states.x - food_pos[i][0]), abs(self.states.y - food_pos[i][1]))
                    if distance < min_distance:
                        min_distance = distance
                        min_distance_index = i
            if min_distance_index > -1:
                self.target_food_x = food_pos[min_distance_index][0]
                self.target_food_y = food_pos[min_distance_index][1]
                self.fsm_state = FSMState.APPROACH
            else:
                self.fsm_state = FSMState.GO_TO_INIT

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            else: 
                if self.states.x == self.target_food_x and self.states.y == self.target_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.GO_HOME
                    keep_executing = False
                else:
                    debugPrint("move towards nearest food")
                    chosen_action = moveToGoal(self.target_food_x, self.target_food_y, self.states.x, self.states.y)
                    chosen_action = obstacleAvoidance(chosen_action, self.submap)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            if isAtHome(self.states.x, self.states.y, home_region):
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
                else:
                    chosen_action = Actions.STAY
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False
        
        elif self.fsm_state == FSMState.GO_TO_INIT:
            debugPrint("fsm_state: GO_TO_INIT")
            if self.states.battery > 1:
                chosen_action = moveToGoal(self.init_pos[0], self.init_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
            else:
                chosen_action = Actions.STAY
            self.fsm_state = FSMState.GO_TO_INIT
            keep_executing = False

    return chosen_action


def uncertainGrabFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = self.constants["battery_size"] // 2

    map_shape = self.constants["map_shape"]
    home_pos = self.constants["home_pos"]
    home_region = constants["home_region"]
    num_food = self.constants["num_food"]
    food_pos = self.constants["food_pos"]
    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SELECT_TARGET:
            debugPrint("fsm_state: SELECT_TARGET")
            self.fsm_failed_grab_attempts = 0
            food_map = getFoodMapFromBinary(self.states.food_state, num_food, food_pos, map_shape)
            min_distance = float('inf')
            min_distance_index = -1
            for i in range(self.constants["num_food"]):
                exclude = False
                if food_map[food_pos[i][0], food_pos[i][1]]:
                    for j in range(len(self.fsm_failed_food_locations)):
                        if food_pos[i][0] == self.fsm_failed_food_locations[j]["x"] and food_pos[i][1] == self.fsm_failed_food_locations[j]["y"]:
                            exclude = True
                    distance = max(abs(self.states.x - food_pos[i][0]), abs(self.states.y - food_pos[i][1]))
                    # TEMP *****************
                    exclude = False
                    # *********************
                    if distance < min_distance and not exclude:
                        min_distance = distance
                        min_distance_index = i
            if min_distance_index > -1:
                self.target_food_x = food_pos[min_distance_index][0]
                self.target_food_y = food_pos[min_distance_index][1]
                self.fsm_state = FSMState.APPROACH
            else:
                self.fsm_state = FSMState.GO_TO_INIT

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            else: 
                if self.states.x == self.target_food_x and self.states.y == self.target_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.CONFIRM_COLLECT
                    keep_executing = False
                else:
                    debugPrint("move towards nearest food")
                    chosen_action = moveToGoal(self.target_food_x, self.target_food_y, self.states.x, self.states.y)
                    chosen_action = obstacleAvoidance(chosen_action, self.submap)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            self.fsm_failed_grab_attempts = 0
            if isAtHome(self.states.x, self.states.y, home_region):
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
                else:
                    chosen_action = Actions.STAY
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
            if self.states.has_food:
                debugPrint("food picked up, going home")
                self.fsm_state = FSMState.GO_HOME
            else:
                debugPrint("food pickup failed")
                self.fsm_failed_grab_attempts += 1
                if self.fsm_failed_grab_attempts >= 2:
                    self.fsm_failed_food_locations.append({"x" : self.states.x, "y" : self.states.y})
                    self.failed_grab_attempts = 0
                    self.fsm_state = FSMState.SELECT_TARGET

        elif self.fsm_state == FSMState.GO_TO_INIT:
            debugPrint("fsm_state: GO_TO_INIT")
            if self.states.battery > 1:
                chosen_action = moveToGoal(self.init_pos[0], self.init_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
            else:
                chosen_action = Actions.STAY
            self.fsm_state = FSMState.GO_TO_INIT
            keep_executing = False

    return chosen_action


def uncertainGrabRandomSelectFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = self.constants["battery_size"] // 2
    
    rng = np.random.default_rng()
    map_shape = self.constants["map_shape"]
    home_pos = self.constants["home_pos"]
    home_region = constants["home_region"]
    num_food = self.constants["num_food"]
    food_pos = self.constants["food_pos"]
    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SELECT_TARGET:
            debugPrint("fsm_state: SELECT_TARGET")
            self.fsm_failed_grab_attempts = 0
            food_map = getFoodMapFromBinary(self.states.food_state, num_food, food_pos, map_shape)
            food_pmf = np.zeros(self.constants["num_food"])
            for i in range(self.constants["num_food"]):
                exclude = False
                if food_map[food_pos[i][0], food_pos[i][1]]:
                    for j in range(len(self.fsm_failed_food_locations)):
                        if food_pos[i][0] == self.fsm_failed_food_locations[j]["x"] and food_pos[i][1] == self.fsm_failed_food_locations[j]["y"]:
                            exclude = True
                    distance = float(max(abs(self.states.x - food_pos[i][0]), abs(self.states.y - food_pos[i][1])))
                    # TEMP *****************
                    exclude = False
                    # *********************
                    if not exclude:
                        if distance > 0.0:
                            food_pmf[i] = 1.0/(distance + 1.0)
                        else:
                            food_pmf[i] = 1.0
            food_pmf_sum = np.sum(food_pmf)
            if food_pmf_sum > 0.0:
                food_pmf /= food_pmf_sum
                selected_food_index = rng.choice(list(range(self.constants["num_food"])), p=food_pmf)
                self.target_food_x = food_pos[selected_food_index][0]
                self.target_food_y = food_pos[selected_food_index][1]
                self.fsm_state = FSMState.APPROACH
            else:
                self.fsm_state = FSMState.GO_TO_INIT

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            food_map = getFoodMapFromBinary(self.states.food_state, num_food, food_pos, map_shape)
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            elif food_map[self.target_food_x, self.target_food_y] == 0: # Another robot grabbed the target food before arriving at the target food location
                self.fsm_state = FSMState.SELECT_TARGET
            else: 
                if self.states.x == self.target_food_x and self.states.y == self.target_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.CONFIRM_COLLECT
                    keep_executing = False
                else:
                    debugPrint("move towards nearest food")
                    chosen_action = moveToGoal(self.target_food_x, self.target_food_y, self.states.x, self.states.y)
                    chosen_action = obstacleAvoidance(chosen_action, self.submap)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            self.fsm_failed_grab_attempts = 0
            if isAtHome(self.states.x, self.states.y, home_region):
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
                else:
                    chosen_action = Actions.STAY
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
            if self.states.has_food:
                debugPrint("food picked up, going home")
                self.fsm_state = FSMState.GO_HOME
            else:
                debugPrint("food pickup failed")
                self.fsm_failed_grab_attempts += 1
                if self.fsm_failed_grab_attempts >= 2:
                    self.fsm_failed_food_locations.append({"x" : self.states.x, "y" : self.states.y})
                    self.failed_grab_attempts = 0
                    self.fsm_state = FSMState.SELECT_TARGET

        elif self.fsm_state == FSMState.GO_TO_INIT:
            debugPrint("fsm_state: GO_TO_INIT")
            if self.states.battery > 1:
                chosen_action = moveToGoal(self.init_pos[0], self.init_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
            else:
                chosen_action = Actions.STAY
            self.fsm_state = FSMState.GO_TO_INIT
            keep_executing = False

    return chosen_action


def uncertainGrabRandomSelectLocalInteractionFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = self.constants["battery_size"] // 2
    
    rng = np.random.default_rng()
    map_shape = self.constants["map_shape"]
    home_pos = self.constants["home_pos"]
    home_region = constants["home_region"]
    num_food = self.constants["num_food"]
    food_pos = self.constants["food_pos"]
    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SELECT_TARGET:
            debugPrint("fsm_state: SELECT_TARGET")
            self.fsm_failed_grab_attempts = 0
            food_map = getFoodMapFromBinary(self.states.food_state, num_food, food_pos, map_shape)
            food_pmf = np.zeros(self.constants["num_food"])
            use_local_influence = False
            other_robot_properties = listVisibleRobotProperties(self.submap, self.constants["personality"])
            other_robot_food_cluster_total = np.zeros(self.constants["num_clusters"])
            for i in range(len(other_robot_properties)):
                if other_robot_properties[i]["has_food"]:
                    use_local_influence = True
                    other_robot_food_cluster_total[other_robot_properties[i]["food_cluster"]] += 1
            for i in range(self.constants["num_food"]):
                exclude = False
                if food_map[food_pos[i][0], food_pos[i][1]]:
                    for j in range(len(self.fsm_failed_food_locations)):
                        if food_pos[i][0] == self.fsm_failed_food_locations[j]["x"] and food_pos[i][1] == self.fsm_failed_food_locations[j]["y"]:
                            exclude = True
                    distance = float(max(abs(self.states.x - food_pos[i][0]), abs(self.states.y - food_pos[i][1])))
                    # TEMP *****************
                    exclude = False
                    # *********************
                    if not exclude:
                        if distance > 0.0:
                            if use_local_influence:
                                preferred_cluster = np.argmax(other_robot_food_cluster_total)
                                if self.constants["food_cluster"][i] == preferred_cluster:
                                    food_pmf[i] = 30.0/(distance + 1.0)
                                else:
                                    food_pmf[i] = 1.0/(distance + 1.0)
                            else:
                                food_pmf[i] = 1.0/(distance + 1.0)
                        else:
                            if use_local_influence:
                                preferred_cluster = np.argmax(other_robot_food_cluster_total)
                                if self.constants["food_cluster"][i] == preferred_cluster:
                                    food_pmf[i] = 30.0
                                else:
                                    food_pmf[i] = 1.0
                            else:
                                food_pmf[i] = 1.0
            food_pmf_sum = np.sum(food_pmf)
            if food_pmf_sum > 0.0:
                food_pmf /= food_pmf_sum
                selected_food_index = rng.choice(list(range(self.constants["num_food"])), p=food_pmf)
                self.target_food_x = food_pos[selected_food_index][0]
                self.target_food_y = food_pos[selected_food_index][1]
                self.fsm_state = FSMState.APPROACH
            else:
                self.fsm_state = FSMState.GO_TO_INIT

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            food_map = getFoodMapFromBinary(self.states.food_state, num_food, food_pos, map_shape)
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            elif food_map[self.target_food_x, self.target_food_y] == 0: # Another robot grabbed the target food before arriving at the target food location
                self.fsm_state = FSMState.SELECT_TARGET
            else: 
                if self.states.x == self.target_food_x and self.states.y == self.target_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.CONFIRM_COLLECT
                    keep_executing = False
                else:
                    debugPrint("move towards nearest food")
                    chosen_action = moveToGoal(self.target_food_x, self.target_food_y, self.states.x, self.states.y)
                    chosen_action = obstacleAvoidance(chosen_action, self.submap)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            self.fsm_failed_grab_attempts = 0
            if isAtHome(self.states.x, self.states.y, home_region):
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
                else:
                    chosen_action = Actions.STAY
                    self.fsm_state = FSMState.SELECT_TARGET
                    keep_executing = False
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
            if self.states.has_food:
                debugPrint("food picked up, going home")
                self.fsm_state = FSMState.GO_HOME
            else:
                debugPrint("food pickup failed")
                self.fsm_failed_grab_attempts += 1
                if self.fsm_failed_grab_attempts >= 2:
                    self.fsm_failed_food_locations.append({"x" : self.states.x, "y" : self.states.y})
                    self.failed_grab_attempts = 0
                    self.fsm_state = FSMState.SELECT_TARGET

        elif self.fsm_state == FSMState.GO_TO_INIT:
            debugPrint("fsm_state: GO_TO_INIT")
            if self.states.battery > 1:
                chosen_action = moveToGoal(self.init_pos[0], self.init_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
            else:
                chosen_action = Actions.STAY
            self.fsm_state = FSMState.GO_TO_INIT
            keep_executing = False

    return chosen_action

def searchFSMActionPolicy(self, enable_local_influence):
    # Constants
    battery_go_home_threshold = self.constants["battery_size"] // 2
    debugPrint("\nrobot id: {0}".format(self.constants["id"]))
    debugPrint("last_successful_approach_dir: {0}".format(self.states.last_approach_dir))
    debugPrint("avg_food_distance: {0}".format(self.avg_food_distance))
    if enable_local_influence and isRobotVisible(self.submap, self.constants["personality"]):
        # If robot of the same personality is visible, record parameters for local influence
        other_robot_properties = listVisibleRobotProperties(self.submap, self.constants["personality"])
        for i in range(len(other_robot_properties)):
            if other_robot_properties[i]["has_food"]:
                # If so, record info to be used for local influence
                self.use_local_influence = True
                if not other_robot_properties[i]["id"] in self.fsm_other_robot_id:
                    self.fsm_other_robot_id.append(other_robot_properties[i]["id"])
                    self.fsm_other_robot_last_successful_food_x.append(other_robot_properties[i]["last_successful_food_x"])
                    self.fsm_other_robot_last_successful_food_y.append(other_robot_properties[i]["last_successful_food_y"])
                    self.fsm_other_robot_last_failed_food_x.append(other_robot_properties[i]["last_failed_food_x"])
                    self.fsm_other_robot_last_failed_food_y.append(other_robot_properties[i]["last_failed_food_y"])
                    self.fsm_other_robot_approach_dir.append(other_robot_properties[i]["last_approach_dir"])

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SEARCH:
            debugPrint("fsm_state: SEARCH")
            self.fsm_reached_approach_location = False
            food_visible = isFoodVisible(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
            max_value_visible = 0.0
            max_value_elsewhere = 0.0
            visible_food_x = []
            visible_food_y = []
            if food_visible:
                (visible_food_delta_x, visible_food_delta_y) = listVisibleFood(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
                num_visible_food = len(visible_food_delta_x)
                for i in range(num_visible_food):
                    visible_food_x.append(visible_food_delta_x[i] + self.states.x)
                    visible_food_y.append(visible_food_delta_y[i] + self.states.y)

            self.grab_prob_map = np.ones(self.map_shape, dtype=np.float)
            self.grab_prob_map /= float(np.size(self.grab_prob_map))
            self.distance_weighted_grab_prob_map = np.ones_like(self.grab_prob_map)
            self.search_goal_prob_map = np.ones_like(self.grab_prob_map)
            for x in range(self.map_shape[0]):
                for y in range(self.map_shape[1]):
                    if self.states.last_successful_food_x > 0 and self.states.last_successful_food_y > 0:
                        distance = max(abs(x - self.states.last_successful_food_x), abs(y - self.states.last_successful_food_y))
                        self.grab_prob_map[x, y] /= (abs(float(distance) - self.avg_food_distance) + 1.0)
                    if self.states.last_failed_food_x > 0 and self.states.last_failed_food_y > 0:
                        distance = max(abs(x - self.states.last_failed_food_x), abs(y - self.states.last_failed_food_y))
                        self.grab_prob_map[x, y] *= (abs(float(distance) - self.avg_food_distance) + 1.0)
                        if self.grab_prob_map[x, y] <= 0.0:
                            self.grab_prob_map[x, y] = 1e-6 # Do not make let any grid cells have zero probability of being chosen
                    if self.use_local_influence:
                        for k in range(len(self.fsm_other_robot_id)):
                            if self.fsm_other_robot_last_successful_food_x[k] > 0 and self.fsm_other_robot_last_successful_food_y[k] > 0:
                                distance = max(abs(x - self.fsm_other_robot_last_successful_food_x[k]), abs(y - self.fsm_other_robot_last_successful_food_y[k]))
                                self.grab_prob_map[x, y] /= (abs(float(distance) - self.avg_food_distance) + 1.0)
                            if self.fsm_other_robot_last_failed_food_x[k] > 0 and self.fsm_other_robot_last_failed_food_y[k] > 0:
                                distance = max(abs(x - self.fsm_other_robot_last_failed_food_x[k]), abs(y - self.fsm_other_robot_last_failed_food_y[k]))
                                self.grab_prob_map[x, y] *= (abs(float(distance) - self.avg_food_distance) + 1.0)
                                if self.grab_prob_map[x, y] <= 0.0:
                                    self.grab_prob_map[x, y] = 1e-6 # Do not make let any grid cells have zero probability of being chosen
                    distance = max(abs(x - self.states.x), abs(y - self.states.y))
                    if distance > 0:
                        self.distance_weighted_grab_prob_map[x, y] = self.grab_prob_map[x, y] / float(distance)
                    else:
                        self.distance_weighted_grab_prob_map[x, y] = self.grab_prob_map[x, y]
                    if distance <= self.constants["perception_range"]:
                        self.search_goal_prob_map[x, y] = 0.0
                    else:
                        self.search_goal_prob_map[x, y] = self.distance_weighted_grab_prob_map[x, y]
                    if (x in visible_food_x) and (y in visible_food_y):
                        if self.grab_prob_map[x, y] > max_value_visible:
                            max_value_visible = self.grab_prob_map[x, y]
                    elif distance > self.constants["perception_range"]:
                        if self.distance_weighted_grab_prob_map[x, y] > max_value_elsewhere:
                            max_value_elsewhere = self.distance_weighted_grab_prob_map[x, y]
                    #self.grab_prob_map[x, y] *= chi2.pdf(float(distance), df=int(self.map_shape[0] / 2))
            self.grab_prob_map /= np.sum(self.grab_prob_map)
            self.distance_weighted_grab_prob_map /= np.sum(self.distance_weighted_grab_prob_map)
            self.search_goal_prob_map /= np.sum(self.search_goal_prob_map)

            if config.enable_action_policy_plots:
                grab_prob_ax.cla()
                search_goal_ax.cla()
                grab_prob_img = grab_prob_ax.imshow(np.swapaxes(self.grab_prob_map, 0, 1), origin='lower', vmin=0.0, vmax=0.05)
                search_goal_img = search_goal_ax.imshow(np.swapaxes(self.search_goal_prob_map, 0, 1), origin='lower', vmin=0.0, vmax=0.05)
                grab_prob_ax.set_xticks([])
                grab_prob_ax.set_xticks([], minor=True)
                grab_prob_ax.set_yticks([])
                grab_prob_ax.set_yticks([], minor=True)
                search_goal_ax.set_xticks([])
                search_goal_ax.set_xticks([], minor=True)
                search_goal_ax.set_yticks([])
                search_goal_ax.set_yticks([], minor=True)
                plt.show()
                plt.waitforbuttonpress()

            self.fsm_approach_target_food_selected = False
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                debugPrint("battery is low, go home")
                self.fsm_failed_search_attempts = 0
                self.fsm_state = FSMState.GO_HOME
            #elif atEdgeOfMap(self.states.x, self.states.y, self.map_shape) and not isAtHome(self.states.x, self.states.y, self.home_pos): # If at edge of map, go home
            #    self.fsm_state = FSMState.GO_HOME
            elif food_visible and max_value_visible >= max_value_elsewhere: # If visible food has a higher value than other possible food elsewhere, approach
                debugPrint("visible food higher value, approach")
                self.fsm_failed_search_attempts = 0
                self.fsm_state = FSMState.APPROACH
            else: # Else, select a search move action
                debugPrint("choose a search goal location")
                if not self.fsm_search_goal_chosen:
                    flat_map = self.search_goal_prob_map.flatten()
                    search_rng = np.random.default_rng()
                    flat_index = search_rng.choice(a=flat_map.size, p=flat_map)
                    (self.search_goal_x, self.search_goal_y) = np.unravel_index(flat_index, self.search_goal_prob_map.shape)
                    debugPrint("search, goal x,y: [{0},{1}]".format(self.search_goal_x, self.search_goal_y))
                    self.fsm_search_goal_chosen = True
                else:
                    if self.states.x == self.search_goal_x and self.states.y == self.search_goal_y:
                        self.fsm_search_goal_chosen = False
                        self.fsm_failed_search_attempts += 1
                        self.fsm_failed_grab_attempts = 0
                        if self.fsm_failed_search_attempts >= 5:
                            self.fsm_failed_search_attempts = 0
                            self.states.last_successful_food_x = -1
                            self.states.last_successful_food_y = -1
                            self.states.last_failed_food_x = -1
                            self.states.last_failed_food_y = -1
                            #avg_food_rng = np.random.default_rng()
                            #possible_avg_food_distance = np.arange(1, min(self.map_shape[0], self.map_shape[1]), dtype=np.int)
                            #self.avg_food_distance = avg_food_rng.choice(possible_avg_food_distance)
                    else:
                        chosen_action = moveToGoal(self.search_goal_x, self.search_goal_y, self.states.x, self.states.y)
                        chosen_action = obstacleAvoidance(chosen_action, self.submap)
                        self.fsm_state = FSMState.SEARCH
                        keep_executing = False
        
        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            self.fsm_search_goal_chosen = False
            self.fsm_failed_search_attempts = 0
            if self.fsm_approach_target_food_selected == False:
                debugPrint("list visible food")
                (food_delta_x, food_delta_y) = listVisibleFood(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
                num_visible_food = len(food_delta_x)
                debugPrint("num_visible_food: {0}".format(num_visible_food))
                if num_visible_food == 0:
                    blocked_moves = findBlockedMoves(self.submap)
                    pmf = removeBlockedMovesFromPMF(MovePMFs.uniform, blocked_moves)
                    elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                    rng = np.random.default_rng()
                    chosen_action = rng.choice(elements, 1, p=pmf)
                    self.fsm_state = FSMState.SEARCH
                    keep_executing = False
                else:
                    # Directional approach logic
                    if self.states.last_approach_dir >= 0:
                        debugPrint("Last approach dir successful: {0}".format(self.states.last_approach_dir))
                        approach_dir = self.states.last_approach_dir
                    else:
                        elements = [Direction.E, Direction.NE, Direction.N, Direction.NW, Direction.W, Direction.SW, Direction.S, Direction.SE]
                        pmf = np.ones(len(elements), dtype=np.float)
                        if self.use_local_influence:
                            for k in range(len(self.fsm_other_robot_id)):
                                if self.fsm_other_robot_approach_dir[k] >= 0:
                                    pmf[self.fsm_other_robot_approach_dir[k]-1] += 1.0
                        pmf /= np.sum(pmf)
                        rng = np.random.default_rng()
                        approach_dir = rng.choice(elements, 1, p=pmf)
                        #approach_dir = 3 # TODO: for testing
                        self.states.last_approach_dir = approach_dir
                        debugPrint("Random approach dir: {0}".format(approach_dir))
                    (approach_offset_delta_x, approach_offset_delta_y) = getDeltaFromDirection(approach_dir)
                    # Select food to approach randomly from list of visible food
                    if num_visible_food > 1:
                        target_food_rng = np.random.default_rng()
                        target_food_index = target_food_rng.choice(range(num_visible_food))
                    else:
                        target_food_index = 0
                    self.target_food_x = food_delta_x[target_food_index] + self.states.x
                    self.target_food_y = food_delta_y[target_food_index] + self.states.y
                    self.approach_goal_x = self.target_food_x + (approach_offset_delta_x * config.robot_footprint_radius)
                    self.approach_goal_y = self.target_food_y + (approach_offset_delta_y * config.robot_footprint_radius)
                    if self.approach_goal_x < 0 or self.approach_goal_x >= self.map_shape[0] or self.approach_goal_y < 0 or self.approach_goal_y >= self.map_shape[1]:
                        debugPrint("approach goal outside map")
                        self.fsm_failed_food_locations.append({"x" : self.target_food_x, "y" : self.target_food_y})
                        self.states.last_approach_dir = -1
                        self.fsm_state = FSMState.SEARCH 
                    else:
                        self.fsm_approach_target_food_selected = True
                        self.fsm_state = FSMState.APPROACH
            else:
                if self.states.x == self.approach_goal_x and self.states.y == self.approach_goal_y and not self.fsm_reached_approach_location:
                    self.fsm_reached_approach_location = True
                    self.fsm_state = FSMState.APPROACH
                elif self.fsm_reached_approach_location and not (self.states.x == self.target_food_x and self.states.y == self.target_food_y):
                    debugPrint("reached approach location, move to food")
                    if isObstacleAtPos(self.target_food_x - self.states.x, self.target_food_y - self.states.y, self.submap):
                        debugPrint("other robot at food location")
                        self.fsm_failed_food_locations.append({"x" : self.target_food_x, "y" : self.target_food_y})
                        self.fsm_state = FSMState.SEARCH 
                    else:
                        chosen_action = moveToGoal(self.target_food_x, self.target_food_y, self.states.x, self.states.y)
                        chosen_action = obstacleAvoidance(chosen_action, self.submap)
                        self.fsm_state = FSMState.APPROACH
                        keep_executing = False
                elif self.states.x == self.target_food_x and self.states.y == self.target_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_approach_target_food_selected = False
                    self.fsm_state = FSMState.CONFIRM_COLLECT
                    keep_executing = False
                elif isObstacleAtPos(self.approach_goal_x - self.states.x, self.approach_goal_y - self.states.y, self.submap):
                    debugPrint("other robot at approach location")
                    self.fsm_failed_food_locations.append({"x" : self.target_food_x, "y" : self.target_food_y})
                    self.fsm_state = FSMState.SEARCH
                    #blocked_moves = findBlockedMoves(self.submap)
                    #pmf = removeBlockedMovesFromPMF(MovePMFs.uniform, blocked_moves)
                    #elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                    #rng = np.random.default_rng()
                    #chosen_action = rng.choice(elements, 1, p=pmf)
                    #self.fsm_state = FSMState.SEARCH
                    #keep_executing = False
                else:
                    debugPrint("move towards target food approach location")
                    chosen_action = moveToGoal(self.approach_goal_x, self.approach_goal_y, self.states.x, self.states.y)
                    temp_submap = copy.deepcopy(self.submap)
                    obstacle_offset_deltas = list(range(-(config.robot_footprint_radius-1), config.robot_footprint_radius))
                    for x_offset in obstacle_offset_deltas:
                        for y_offset in obstacle_offset_deltas:
                            temp_submap[0].append(MapLayer.OBSTACLE)
                            temp_submap[1].append({"delta_x" : self.target_food_x + x_offset - self.states.x, "delta_y" : self.target_food_y + y_offset - self.states.y})
                    chosen_action = obstacleAvoidance(chosen_action, temp_submap)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            self.fsm_search_goal_chosen = False
            self.fsm_approach_target_food_selected = False
            self.fsm_reached_approach_location = False
            self.fsm_failed_grab_attempts = 0
            self.fsm_failed_food_locations = []
            self.use_local_influence = False
            self.resetOtherRobotLists()
            if isAtHome(self.states.x, self.states.y, self.home_region):
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.GO_TO_INIT
                    keep_executing = False
                else:
                    chosen_action = Actions.STAY
                    self.fsm_state = FSMState.GO_TO_INIT
                    keep_executing = False
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
            self.fsm_search_goal_chosen = False
            self.fsm_approach_target_food_selected = False
            self.fsm_reached_approach_location = False
            if self.states.has_food:
                debugPrint("food picked up, going home")
                self.fsm_failed_grab_attempts = 0
                # TODO: temporary solution for passing internal robot data out to the true states for local interacion data communication
                self.states.last_successful_food_x = self.states.x
                self.states.last_successful_food_y = self.states.y
                if max(abs(self.states.last_successful_food_x - self.states.last_failed_food_x), \
                        abs(self.states.last_successful_food_y - self.states.last_failed_food_y)) <= self.constants["perception_range"]:
                    self.states.last_failed_food_x = -1
                    self.states.last_failed_food_y = -1
                self.states.last_approach_dir = self.states.last_approach_dir
                # ----------------------------------------------------------------------------------------------------------------------
                self.fsm_state = FSMState.GO_HOME
            else:
                debugPrint("food pickup failed")
                self.states.last_approach_dir = -1
                self.fsm_failed_grab_attempts += 1
                self.states.last_failed_food_x = self.states.x
                self.states.last_failed_food_y = self.states.y
                if max(abs(self.states.last_successful_food_x - self.states.last_failed_food_x), \
                        abs(self.states.last_successful_food_y - self.states.last_failed_food_y)) <= self.constants["perception_range"]:
                    self.states.last_successful_food_x = -1
                    self.states.last_successful_food_y = -1
                self.fsm_failed_food_locations.append({"x" : self.states.x, "y" : self.states.y})
                #self.fsm_failed_grab_attempts = 0
                self.fsm_state = FSMState.SEARCH
        
        elif self.fsm_state == FSMState.GO_TO_INIT:
            debugPrint("fsm_state: GO_TO_INIT")
            if self.states.x == self.init_pos[0] and self.states.y == self.init_pos[1]:
                self.fsm_state = FSMState.SEARCH
            else:
                chosen_action = moveToGoal(self.init_pos[0], self.init_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_TO_INIT
                keep_executing = False


    return chosen_action


def mmMDPMajorityVotingActionPolicy(self):
    votes = np.zeros(self.num_actions, dtype=np.int)
    state_index = enumerateState(self.states, self.state_dimensions)
    for i in range(self.num_models):
        action = self.policy[i][state_index]
        votes[action] += 1

    return np.argmax(votes)


def mmMDPWeightedMajorityVotingActionPolicy(self):
    votes = np.zeros(self.num_actions, dtype=np.float)
    state_index = enumerateState(self.states, self.state_dimensions)
    for i in range(self.num_models):
        action = self.policy[i][state_index]
        votes[action] += self.bum[i]

    return np.argmax(votes)


def mmMDPHighestPreferenceActionPolicy(self):
    preference = np.zeros(self.num_actions, dtype=np.float)
    state_index = enumerateState(self.states, self.state_dimensions)
    for i in range(self.num_models):
        for j in range(self.num_actions):
            preference[j] += self.Q[i][state_index, j]

    return np.argmax(preference)


def mmMDPWeightedHighestPreferenceActionPolicy(self):
    preference = np.zeros(self.num_actions, dtype=np.float)
    state_index = enumerateState(self.states, self.state_dimensions)
    for i in range(self.num_models):
        for j in range(self.num_actions):
            preference[j] += self.Q[i][state_index, j] * self.bum[i]

    return np.argmax(preference)


def moveToGoal(goal_x, goal_y, x, y):
    if goal_x > x and goal_y == y:
        return Actions.MOVE_E
    elif goal_x > x and goal_y > y:
        return Actions.MOVE_NE
    elif goal_x == x and goal_y > y:
        return Actions.MOVE_N
    elif goal_x < x and goal_y > y:
        return Actions.MOVE_NW
    elif goal_x < x and goal_y == y:
        return Actions.MOVE_W
    elif goal_x < x and goal_y < y:
        return Actions.MOVE_SW
    elif goal_x == x and goal_y < y:
        return Actions.MOVE_S
    elif goal_x > x and goal_y < y:
        return Actions.MOVE_SE
    else:
        return Actions.STAY

def actionToDirection(action):
    if action == Actions.MOVE_E:
        return Direction.E
    elif action == Actions.MOVE_NE:
        return Direction.NE
    elif action == Actions.MOVE_N:
        return Direction.N
    elif action == Actions.MOVE_NW:
        return Direction.NW
    elif action == Actions.MOVE_W:
        return Direction.W
    elif action == Actions.MOVE_SW:
        return Direction.SW
    elif action == Actions.MOVE_S:
        return Direction.S
    elif action == Actions.MOVE_SE:
        return Direction.SE
    else:
        return Direction.NONE

def directionToAction(direction):
    if direction == Direction.E:
        return Actions.MOVE_E
    elif direction == Direction.NE:
        return Actions.MOVE_NE
    elif direction == Direction.N:
        return Actions.MOVE_N
    elif direction == Direction.NW:
        return Actions.MOVE_NW
    elif direction == Direction.W:
        return Actions.MOVE_W
    elif direction == Direction.SW:
        return Actions.MOVE_SW
    elif direction == Direction.S:
        return Actions.MOVE_S
    elif direction == Direction.SE:
        return Actions.MOVE_SE
    else:
        return Direction.NONE

def removeBlockedMovesFromPMF(pmf, blocked_moves):
    new_pmf = pmf
    num_moves = len(pmf)
    excluded_indices = np.array([])
    for i in range(len(blocked_moves)):
        index = blocked_moves[i] - 1
        move_prob = pmf[index]
        new_pmf[index] = 0.0
        np.append(excluded_indices, index)
        num_excluded = excluded_indices.shape[0]
        for j in range(num_moves):
            if j not in excluded_indices:
                new_pmf[j] += move_prob / float(num_moves - num_excluded)

    new_pmf_sum = np.sum(new_pmf)
    new_pmf /= new_pmf_sum
    return new_pmf

def obstacleAvoidance(chosen_action, submap):
    move_dir = actionToDirection(chosen_action)
    blocked_moves = findBlockedMoves(submap)
    if move_dir in blocked_moves:
        if config.use_bug_avoidance:
            keep_checking = True
            initial_dir = move_dir
            candidate_dir = move_dir
            while keep_checking:
                candidate_dir -= 1
                if candidate_dir < 1:
                    candidate_dir = 8
                if not (candidate_dir in blocked_moves):
                    chosen_action = directionToAction(move_dir)
                    keep_checking = False
                if candidate_dir == initial_dir:
                    elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                    rng = np.random.default_rng()
                    chosen_action = rng.choice(elements, 1)
                    keep_checking = False
                debugPrint("blocked_moves: {0}".format(blocked_moves))
                debugPrint("candidate_dir: {0}".format(candidate_dir))
        else:
            pmf = np.ones(8) / 8.0
            pmf = removeBlockedMovesFromPMF(pmf, blocked_moves)
            elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
            rng = np.random.default_rng()
            chosen_action = rng.choice(elements, 1, p=pmf)
    return chosen_action
