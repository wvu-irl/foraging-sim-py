from enum import IntEnum, unique
from actions import Actions
from submap_utils import *
import numpy as np

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
            print("fsm_state: SEARCH")
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
                #chosen_action = Actions.MOVE_NE
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            print("fsm_state: APPROACH")
            if self.fsm_nearest_food_found == False:
                print("find nearest food")
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap)
                self.nearest_food_x = nearest_delta_x + self.states.x
                self.nearest_food_y = nearest_delta_y + self.states.y
                self.fsm_nearest_food_found = True
                self.fsm_state = FSMState.APPROACH
            else:
                if self.states.x == self.nearest_food_x and self.states.y == self.nearest_food_y:
                    print("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.GO_HOME
                    keep_executing = False
                else:
                    print("move towards nearest food")
                    chosen_action = moveToGoal(self.nearest_food_x, self.nearest_food_y, self.states.x, self.states.y)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            print("fsm_state: GO_HOME")
            self.fsm_nearest_food_found = False
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SEARCH
                    keep_executing = False
                else:
                    self.fsm_state = FSMState.SEARCH
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
            print("fsm_state: SEARCH")
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
                #chosen_action = Actions.MOVE_NE
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            print("fsm_state: APPROACH")
            if self.fsm_nearest_food_found == False:
                print("find nearest food")
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap, self.fsm_failed_food_locations, self.states.x, self.states.y)
                self.nearest_food_x = nearest_delta_x + self.states.x
                self.nearest_food_y = nearest_delta_y + self.states.y
                self.fsm_nearest_food_found = True
                self.fsm_state = FSMState.APPROACH
            else:
                if self.states.x == self.nearest_food_x and self.states.y == self.nearest_food_y:
                    print("grab")
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.CONFIRM_COLLECT
                    keep_executing = False
                else:
                    print("move towards nearest food")
                    chosen_action = moveToGoal(self.nearest_food_x, self.nearest_food_y, self.states.x, self.states.y)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            print("fsm_state: GO_HOME")
            self.fsm_nearest_food_found = False
            self.fsm_failed_grab_attempts = 0
            if self.states.x == self.home_pos[0] and self.states.y == self.home_pos[1]:
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SEARCH
                    keep_executing = False
                else:
                    self.fsm_state = FSMState.SEARCH
            else:
                chosen_action = moveToGoal(self.home_pos[0], self.home_pos[1], self.states.x, self.states.y)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

        elif self.fsm_state == FSMState.CONFIRM_COLLECT:
            print("fsm_state: CONFIRM_COLLECT")
            self.fsm_nearest_food_found = False
            if self.states.has_food:
                print("food picked up, going home")
                self.fsm_state = FSMState.GO_HOME
            else:
                print("food pickup failed")
                self.fsm_failed_grab_attempts += 1
                if self.fsm_failed_grab_attempts >= 2:
                    # TODO mark robot x,y as failed food location, need to use this to exclude in approach
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
