from enum import IntEnum, unique
from actions import Actions
from submap_utils import *
import random

@unique
class FSMState(IntEnum):
    SEARCH = 0
    APPROACH = 1
    GO_HOME = 2

def simpleFSMActionPolicy(self):
    # Constants
    battery_go_home_threshold = 30.0

    keep_executing = True
    while keep_executing:
        if self.fsm_state == FSMState.SEARCH:
            self.fsm_nearest_food_found = False
            if self.states.battery < battery_go_home_threshold: # If battery is below threshold, go home
                self.fsm_state = FSMState.GO_HOME
            elif atEdgeOfMap(self.states.x, self.states.y, self.map_shape): # If at edge of map, go home
                self.fsm_state = FSMState.GO_HOME
            elif isFoodVisible(self.submap): # If food is visible, approach
                self.fsm_state = FSMState.APPROACH
            else: # Else, select a search move action
                chosen_action = random.randint(Actions.MOVE_E, Actions.MOVE_SE)
                self.fsm_state = FSMState.SEARCH
                keep_executing = False

        elif self.fsm_state == FSMState.APPROACH:
            if self.fsm_nearest_food_found == False:
                (nearest_delta_x, nearest_delta_y) = findNearestFood(self.submap)
                nearest_x = nearest_delta_x + self.states.x
                nearest_y = nearest_delta_y + self.states.y
                self.fsm_state = FSMState.APPROACH
            else:
                if self.states.x == nearest_x and self.states.y == nearest_y:
                    chosen_action = Actions.GRAB
                    self.fsm_nearest_food_found = False
                    self.fsm_state = FSMState.GO_HOME
                    keep_executing = False
                else:
                    chosen_action = moveToGoal(nearest_x, nearest_y, self.states.x, self.states.y)
                    self.fsm_state = FSMState.APPROACH
                    keep_executing = False

        elif self.fsm_state == FSMState.GO_HOME:
            self.fsm_nearest_food_found = False
            if self.states.x == self.constants["home_pos"][0] and self.states.y == self.constants["home_pos"][1]:
                if self.states.has_food:
                    chosen_action = Actions.DROP
                    self.fsm_state = FSMState.SEARCH
                    keep_executing = False
                else:
                    self.fsm_state = FSMState.SEARCH
            else:
                chosen_action = moveToGoal(self.constants["home_pos"][0], self.constants["home_pos"][1], self.states.x, self.states.y)
                self.fsm_state = FSMState.GO_HOME
                keep_executing = False

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
