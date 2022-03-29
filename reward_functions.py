from submap_utils import *
from actions import *
import numpy as np

def mdpRewardFunction(state, action, state_prime, constants):
    if state_prime.battery == 0: # Check if battery becomes empty
        r = -1000.0
    elif Actions.MOVE_E <= action <= Actions.MOVE_SE:
        r = -1.0
    elif action == Actions.STAY:
        r = 0.0
    elif action == Actions.GRAB and state_prime.has_food == False:
        r = -20.0
    elif action == Actions.GRAB and state.has_food == False and state_prime.has_food == True:
        r = 20.0
    elif action == Actions.DROP and state.has_food == True and state_prime.has_food == False and state.x == constants["home_pos"][0] and state.y == constants["home_pos"][1]:
        r = 10000.0
    else:
        r = -10000.0
    
    return r
