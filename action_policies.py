from enum import IntEnum, unique
from actions import Actions
from submap_utils import *
import numpy as np
from debug_print import debugPrint

@unique
class FSMState(IntEnum):
    SEARCH = 0
    APPROACH = 1
    GO_HOME = 2
    CONFIRM_COLLECT = 3

def simpleFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = 30.0

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SEARCH:
            debugPrint("fsm_state: SEARCH")
            self.fsm_nearest_food_found = False
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            elif atEdgeOfMap(self.states.x, self.states.y, self.map_shape): # If at edge of map, go home
                self.fsm_state = FSMState.GO_HOME
            elif isFoodVisible(self.submap): # If food is visible, approach
                self.fsm_state = FSMState.APPROACH
            else: # Else, select a search move action
                if self.states.x > self.home_pos[0] and self.states.y > self.home_pos[1]:
                    pmf = np.array([1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000])
                elif self.states.x < self.home_pos[0] and self.states.y > self.home_pos[1]:
                    pmf = np.array([0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000])
                elif self.states.x < self.home_pos[0] and self.states.y < self.home_pos[1]:
                    pmf = np.array([0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000])
                elif self.states.x > self.home_pos[0] and self.states.y < self.home_pos[1]:
                    pmf = np.array([1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0])
                else:
                    pmf = np.ones(8) / 8.0
                elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                rng = np.random.default_rng()
                chosen_action = rng.choice(elements, 1, p=pmf)
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            if self.fsm_nearest_food_found == False:
                debugPrint("find nearest food")
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap)
                self.nearest_food_x = nearest_delta_x + self.states.x
                self.nearest_food_y = nearest_delta_y + self.states.y
                self.fsm_nearest_food_found = True
                self.fsm_state = FSMState.APPROACH
            else:
                if self.states.x == self.nearest_food_x and self.states.y == self.nearest_food_y:
                    debugPrint("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.GO_HOME
                    keep_executing = False
                else:
                    debugPrint("move towards nearest food")
                    chosen_action = moveToGoal(self.nearest_food_x, self.nearest_food_y, self.states.x, self.states.y)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
            self.fsm_nearest_food_found = False
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
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

    return chosen_action


def uncertainGrabFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = 30.0

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SEARCH:
            debugPrint("fsm_state: SEARCH")
            self.fsm_nearest_food_found = False
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            elif atEdgeOfMap(self.states.x, self.states.y, self.map_shape): # If at edge of map, go home
                self.fsm_state = FSMState.GO_HOME
            elif isFoodVisible(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y): # If food is visible, approach
                self.fsm_state = FSMState.APPROACH
            else: # Else, select a search move action
                if self.states.x > self.home_pos[0] and self.states.y > self.home_pos[1]:
                    pmf = np.array([1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000])
                elif self.states.x < self.home_pos[0] and self.states.y > self.home_pos[1]:
                    pmf = np.array([0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000])
                elif self.states.x < self.home_pos[0] and self.states.y < self.home_pos[1]:
                    pmf = np.array([0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000])
                elif self.states.x > self.home_pos[0] and self.states.y < self.home_pos[1]:
                    pmf = np.array([1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0])
                else:
                    pmf = np.ones(8) / 8.0
                elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                rng = np.random.default_rng()
                chosen_action = rng.choice(elements, 1, p=pmf)
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            if self.fsm_nearest_food_found == False:
                debugPrint("find nearest food")
                # TODO: need to handle if this returns no non-excluded food. Otherwise robot goes towards x,y infinity
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
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
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
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
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
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


def uncertainGrabLocalInteractionFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = 30.0

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SEARCH:
            debugPrint("fsm_state: SEARCH")
            self.fsm_nearest_food_found = False
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            elif atEdgeOfMap(self.states.x, self.states.y, self.map_shape): # If at edge of map, go home
                self.fsm_state = FSMState.GO_HOME
            elif isFoodVisible(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y): # If food is visible, approach
                self.fsm_state = FSMState.APPROACH
            else: # Else, select a search move action
                use_local_influence = False
                if isRobotVisible(self.submap, self.states.personality):
                    # If robot of the same personality is visible, check if it is further away from home current location
                    (other_robot_delta_x, other_robot_delta_y, other_robot_has_food) = findNearestRobot(self.submap, self.states.personality)
                    distance_to_home = max(abs(self.states.x - self.home_pos[0]), abs(self.states.y - self.home_pos[1])) # Chebyshev distance
                    other_robot_distance_to_home = max(abs(self.states.x + other_robot_delta_x - self.home_pos[0]), abs(self.states.y + other_robot_delta_y - self.home_pos[1]))
                    if other_robot_distance_to_home > distance_to_home and other_robot_has_food:
                        # If the other robot is farther away from home and has food, record so
                        use_local_influence = True
                if use_local_influence:
                    print("Use local influence")
                    # Use influence from this robot to choose move action
                    if other_robot_delta_x >= 0 and other_robot_delta_y > 0: # First quadrant
                        diag_distance = other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            bias_direction = Direction.E
                        elif min_distance_index == 1:
                            bias_direction = Direction.N
                        elif min_distance_index == 2:
                            bias_direction = Direction.NE
                    elif other_robot_delta_x < 0 and other_robot_delta_y >= 0: # Second quadrant
                        diag_distance = -other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            bias_direction = Direction.W
                        elif min_distance_index == 1:
                            bias_direction = Direction.N
                        elif min_distance_index == 2:
                            bias_direction = Direction.NW
                    elif other_robot_delta_x <= 0 and other_robot_delta_y < 0: # Third quadrant
                        diag_distance = other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            bias_direction = Direction.W
                        elif min_distance_index == 1:
                            bias_direction = Direction.S
                        elif min_distance_index == 2:
                            bias_direction = Direction.SW
                    elif other_robot_delta_x > 0 and other_robot_delta_y <= 0: # Fourth quadrant
                        diag_distance = -other_robot_delta_x - other_robot_delta_y
                        distances = [other_robot_delta_y, other_robot_delta_x, diag_distance]
                        min_distance_index = distances.index(min(distances))
                        if min_distance_index == 0:
                            bias_direction = Direction.E
                        elif min_distance_index == 1:
                            bias_direction = Direction.S
                        elif min_distance_index == 2:
                            bias_direction = Direction.SE

                    # Choose move action based on bias direction
                    if bias_direction == Direction.E:
                         pmf = np.array([2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/4.0])
                    elif bias_direction == Direction.NE:
                         pmf = np.array([1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000, 0.000, 0.000])
                    elif bias_direction == Direction.N:
                         pmf = np.array([0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000, 0.000])
                    elif bias_direction == Direction.NW:
                         pmf = np.array([0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000, 0.000])
                    elif bias_direction == Direction.W:
                         pmf = np.array([0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000, 0.000])
                    elif bias_direction == Direction.SW:
                         pmf = np.array([0.000, 0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0, 0.000])
                    elif bias_direction == Direction.S:
                         pmf = np.array([0.000, 0.000, 0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0, 1.0/4.0])
                    elif bias_direction == Direction.SE:
                         pmf = np.array([1.0/4.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/4.0, 2.0/4.0])

                else:
                    # Choose move action based on quadrant
                    if self.states.x > self.home_pos[0] and self.states.y > self.home_pos[1]:
                        pmf = np.array([1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000])
                    elif self.states.x < self.home_pos[0] and self.states.y > self.home_pos[1]:
                        pmf = np.array([0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000, 0.000, 0.000])
                    elif self.states.x < self.home_pos[0] and self.states.y < self.home_pos[1]:
                        pmf = np.array([0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0, 1.0/3.0, 0.000])
                    elif self.states.x > self.home_pos[0] and self.states.y < self.home_pos[1]:
                        pmf = np.array([1.0/3.0, 0.000, 0.000, 0.000, 0.000, 0.000, 1.0/3.0, 1.0/3.0])
                    else:
                        pmf = np.ones(8) / 8.0
                elements = [Actions.MOVE_E, Actions.MOVE_NE, Actions.MOVE_N, Actions.MOVE_NW, Actions.MOVE_W, Actions.MOVE_SW, Actions.MOVE_S, Actions.MOVE_SE]
                rng = np.random.default_rng()
                chosen_action = rng.choice(elements, 1, p=pmf)
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            debugPrint("fsm_state: APPROACH")
            if self.fsm_nearest_food_found == False:
                debugPrint("find nearest food")
                # TODO: need to handle if this returns no non-excluded food. Otherwise robot goes towards x,y infinity
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
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
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            debugPrint("fsm_state: GO_HOME")
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
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            debugPrint("fsm_state: CONFIRM_COLLECT")
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
