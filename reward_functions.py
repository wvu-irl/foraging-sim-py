from submap_utils import *
from actions import *
import numpy as np

def mdpRewardFunction(state, action, state_prime, constants):
    if state_prime.battery == 0: # Check if battery is empty
        r = -1000.0
    elif state_prime.battery < state.battery and (state.x != state_prime.x or state.y != state_prime.y):
        r = -1.0*(state.battery - state_prime.battery)
    elif state_prime.battery == (constants["battery_size"] - 1) and state_prime.x == constants["home_pos"][0] and state_prime.y == constants["home_pos"][1]:
        r = 0.0
    elif Actions.STAY <= action <= Actions.MOVE_SE and state.battery == state_prime.battery and state.x == state_prime.x and state.y == state_prime.y:
        r = 0.0
    elif action == Actions.GRAB and state_prime.has_food == False and state.x == state_prime.x and state.y == state_prime.y:
        r = -20.0
    elif action == Actions.GRAB and state_prime.has_food == True and state.x == state_prime.x and state.y == state_prime.y:
        r = 20.0
    elif action == Actions.DROP and state.has_food == True and state_prime.has_food == False and state.x == constants["home_pos"][0] and state.y == constants["home_pos"][1] and state.x == state_prime.x and state.y == state_prime.y:
        #print("DROP REWARD")
        r = 100.0
    elif action == Actions.DROP and state.has_food == True and state_prime.has_food == True and not(state.x == constants["home_pos"][0] and state.y == constants["home_pos"][1]) and state.x == state_prime.x and state.y == state_prime.y:
        #print("DROP NOT AT HOME")
        r = -20.0
    elif action == Actions.DROP and state.has_food == False and state_prime.has_food == False and state.x == state_prime.x and state.y == state_prime.y:
        r = -20.0
    else:
        r = -5000.0
    
    return r
