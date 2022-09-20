from submap_utils import *
from actions import *
import numpy as np

def mdpRewardFunction(state, action, state_prime, constants):
    if state_prime.battery == 0 and state.battery > 0: # Check if battery becomes empty
        r = -1000.0
    elif state_prime.battery == 0 and state.battery == 0: # Battery is already dead, terminal state, no need to keep penalizing
        r = 0.0
    elif Actions.MOVE_E <= action <= Actions.MOVE_SE:
        r = -1.0
    elif action == Actions.STAY:
        r = 0.0
    elif action == Actions.GRAB and state_prime.has_food == False:
        r = -20.0
    elif action == Actions.GRAB and state.has_food == False and state_prime.has_food == True:
        r = 20.0
    elif action == Actions.DROP and state.has_food == True and state_prime.has_food == False and state.x in constants["home_region"][0] and state.y in constants["home_region"][1]:
        r = 100.0
    else:
        r = 0.0 # This should stil never occur...
        #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        #print("state: bat: {0}, has_food: {1}, in home region: {2}".format(state.battery, state.has_food, state.x in constants["home_region"][0] and state.y in constants["home_region"][1]))
        #print("action: {0}".format(action))
        #print("state_prime: bat: {0}, has_food: {1}, in home region: {2}".format(state_prime.battery, state_prime.has_food, state_prime.x in constants["home_region"][0] and state_prime.y in constants["home_region"][1]))
        #print("###########################\n")
    
    return r
