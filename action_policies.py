from enum import IntEnum, unique
from actions import Actions
from submap_utils import *
from transition_models import *
import numpy as np
from debug_print import debugPrint

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
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
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
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
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
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
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
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
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

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SEARCH:
            debugPrint("fsm_state: SEARCH")
            self.fsm_nearest_food_found = False
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            #elif atEdgeOfMap(self.states.x, self.states.y, self.map_shape) and not isAtHome(self.states.x, self.states.y, self.home_pos): # If at edge of map, go home
            #    self.fsm_state = FSMState.GO_HOME
            elif isFoodVisible(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y): # If food is visible, approach
                self.fsm_state = FSMState.APPROACH
            else: # Else, select a search move action
                if not self.fsm_search_dir_chosen:
                    self.fsm_search_dir_chosen = True
                    #pmf_elements = [MovePMFs.wide_E, MovePMFs.wide_NE, MovePMFs.wide_N, MovePMFs.wide_NW, MovePMFs.wide_W, MovePMFs.wide_SW, MovePMFs.wide_S, MovePMFs.wide_SE]
                    #pmf_elements = [MovePMFs.wide_E, MovePMFs.wide_NE, MovePMFs.wide_N, MovePMFs.wide_NW]
                    pmf_elements = [MovePMFs.narrow_E, MovePMFs.narrow_NE]
                    rng = np.random.default_rng()
                    self.fsm_search_pmf = rng.choice(pmf_elements)
                use_local_influence = False
                if enable_local_influence and isRobotVisible(self.submap, self.constants["personality"]):
                    # If robot of the same personality is visible, check if it is further away from home current location
                    (other_robot_delta_x, other_robot_delta_y, other_robot_has_food) = findNearestRobot(self.submap, self.constants["personality"])
                    distance_to_home = max(abs(self.states.x - self.home_pos[0]), abs(self.states.y - self.home_pos[1])) # Chebyshev distance
                    other_robot_distance_to_home = max(abs(self.states.x + other_robot_delta_x - self.home_pos[0]), abs(self.states.y + other_robot_delta_y - self.home_pos[1]))
                    if other_robot_distance_to_home > distance_to_home and other_robot_has_food:
                        # If the other robot is farther away from home and has food, record so
                        use_local_influence = True
                if use_local_influence:
                    # Use influence from this robot to choose move action
                    if other_robot_delta_x >= 0 and other_robot_delta_y > 0: # First quadrant
                        diag_distance = other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            self.fsm_search_pmf = MovePMFs.narrow_E
                        elif min_distance_index == 1:
                            self.fsm_search_pmf = MovePMFs.narrow_N
                        elif min_distance_index == 2:
                            self.fsm_search_pmf = MovePMFs.narrow_NE
                    elif other_robot_delta_x < 0 and other_robot_delta_y >= 0: # Second quadrant
                        diag_distance = -other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            self.fsm_search_pmf = MovePMFs.narrow_W
                        elif min_distance_index == 1:
                            self.fsm_search_pmf = MovePMFs.narrow_N
                        elif min_distance_index == 2:
                            self.fsm_search_pmf = MovePMFs.narrow_NW
                    elif other_robot_delta_x <= 0 and other_robot_delta_y < 0: # Third quadrant
                        diag_distance = other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            self.fsm_search_pmf = MovePMFs.narrow_W
                        elif min_distance_index == 1:
                            self.fsm_search_pmf = MovePMFs.narrow_S
                        elif min_distance_index == 2:
                            self.fsm_search_pmf = MovePMFs.narrow_SW
                    elif other_robot_delta_x > 0 and other_robot_delta_y <= 0: # Fourth quadrant
                        diag_distance = -other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            self.fsm_search_pmf = MovePMFs.narrow_E
                        elif min_distance_index == 1:
                            self.fsm_search_pmf = MovePMFs.narrow_S
                        elif min_distance_index == 2:
                            self.fsm_search_pmf = MovePMFs.narrow_SE

                blocked_moves = findBlockedMoves(self.submap)
                pmf = removeBlockedMovesFromPMF(self.fsm_search_pmf, blocked_moves)
                elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                rng = np.random.default_rng()
                chosen_action = rng.choice(elements, 1, p=pmf)
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            self.fsm_search_dir_chosen = False
            if self.fsm_nearest_food_found == False:
                debugPrint("find nearest food")
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
                if nearest_delta_x == sys.maxsize or nearest_delta_y == sys.maxsize:
                    blocked_moves = findBlockedMoves(self.submap)
                    pmf = removeBlockedMovesFromPMF(MovePMFs.uniform, blocked_moves)
                    elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                    rng = np.random.default_rng()
                    chosen_action = rng.choice(elements, 1, p=pmf)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False
                else:
                    self.nearest_food_x = nearest_delta_x + self.states.x
                    self.nearest_food_y = nearest_delta_y + self.states.y
                    self.fsm_nearest_food_found = True
                    self.fsm_state = FSMState.APPROACH
            else:
                if self.states.x == self.nearest_food_x and self.states.y == self.nearest_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.CONFIRM_COLLECT
                    keep_executing = False
                else:
                    debugPrint("move towards nearest food")
                    chosen_action = moveToGoal(self.nearest_food_x, self.nearest_food_y, self.states.x, self.states.y)
                    chosen_action = obstacleAvoidance(chosen_action, self.submap)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            self.fsm_search_dir_chosen = False
            self.fsm_nearest_food_found = False
            self.fsm_failed_grab_attempts = 0
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SEARCH
                    keep_executing = False
                else:
                    chosen_action = Actions.STAY
                    self.fsm_state = FSMState.SEARCH
                    keep_executing = False
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                chosen_action = obstacleAvoidance(chosen_action, self.submap)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
            self.fsm_search_dir_chosen = False
            self.fsm_nearest_food_found = False
            if self.states.has_food:
                debugPrint("food picked up, going home")
                self.fsm_state = FSMState.GO_HOME
            else:
                debugPrint("food pickup failed")
                self.fsm_failed_grab_attempts += 1
                if self.fsm_failed_grab_attempts >= 2:
                    self.fsm_failed_food_locations.append({"x" : self.states.x, "y" : self.states.y})
                    self.failed_grab_attempts = 0
                    self.fsm_state = FSMState.SEARCH


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
        pmf = np.ones(8) / 8.0
        pmf = removeBlockedMovesFromPMF(pmf, blocked_moves)
        elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
        rng = np.random.default_rng()
        chosen_action = rng.choice(elements, 1, p=pmf)
    return chosen_action
