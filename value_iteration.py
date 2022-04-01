import copy
from PIL import Image
import numpy as np
#from multiprocessing import Process, Lock
from states import *
from actions import *
from transition_models import *
from reward_functions import *

from params._1_single_robot_mdp_correct_model import *

output_filename = "policies/single_robot_vi_policy.npy"

# Set transition and reward functions
T = mdpDirectionalFoodTransitionModel
R = mdpRewardFunction

# Set number of parallel threads
num_parallel_threads = 1

# Load map initialization image files
food_img = Image.open(food_img_path)
home_img = Image.open(home_img_path)
obstacle_img = Image.open(obstacle_img_path)
robot_img = Image.open(robot_img_path)

# Convert images to numpy arrays
food_layer = np.array(food_img)
home_layer = np.array(home_img)
obstacle_layer = np.array(obstacle_img)
robot_layer = np.array(robot_img)

# Initialize map
map_obj = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
map_shape = map_obj.map_shape
        
# Record the food positions and headings lists
food_pos, food_heading = map_obj.findFoodInfo()
num_food = len(food_pos)
print("num_food: {0}".format(num_food))
        
# Record home location
home_pos = map_obj.findHomePosition(0)

# Record constants and state dimensions
constants = {"map_shape" : map_shape, "battery_size" : battery_size, "home_pos" : home_pos,"num_food" : num_food, "food_pos" : food_pos}
state_dimensions = {"x_size" : map_shape[0], "y_size" : map_shape[1], "has_food_size" : 2, "battery_size" : battery_size, "num_food" : num_food}
num_states = state_dimensions["x_size"] * state_dimensions["y_size"] * state_dimensions["has_food_size"] * state_dimensions["battery_size"] * (2 ** state_dimensions["num_food"])
num_actions = Actions.DROP + 1
print(state_dimensions)
print("num_states: {0}".format(num_states))
print("num_actions: {0}".format(num_actions))

#val = 0.0
#for s in range(num_states):
#    s_vals = deEnumerateState(s, state_dimensions)
#    for a in range(num_actions):
#        for s_prime in range(num_states):
#            s_prime_vals = deEnumerateState(s_prime, state_dimensions)
#            indiv_val = T(s_vals, a, s_prime_vals, constants) * R(s_vals, a, s_prime_vals, constants)
#            print(indiv_val)
#            val += indiv_val
#
#print(val)


# Set value iteration parameters
max_iter = 10000  # Maximum number of iterations
#delta = 0.0001  # Error tolerance
delta = 1.0
gamma = 0.9 # Discount factor

V = np.zeros(num_states)
pi = np.ones(num_states, dtype=np.int) * -1
iterations = np.zeros(num_states, dtype=np.int)

V_new = np.zeros(num_states)  # Initialize values
for i in range(max_iter):
    print("\ni: {0}".format(i))
    max_diff = 1e-6
    for s in range(num_states):
        #print("s: {0}".format(s))
        s_vals = deEnumerateState(s, state_dimensions)
        max_val = float('-inf')
        possible_actions = list(range(num_actions))
        if s_vals.has_food == True:
            possible_actions.remove(Actions.GRAB)
        if s_vals.has_food == False:
            possible_actions.remove(Actions.DROP)
        if (s_vals.x != home_pos[0] or s_vals.y != home_pos[1]) and s_vals.has_food == True:
            possible_actions.remove(Actions.DROP)
        #print(possible_actions)
        for a in possible_actions:
            #print("a: {0}".format(a))
            # Compute state value
            val = 0.0
            #print("+++++++++++++++++++++++")
            s_prime_vals, T_prob = T(s_vals, a, constants)
            for idx in range(len(s_prime_vals)):
                #print("s_prime: {0}".format(s_prime))
                #print("++++++++++++++++++++++++++++++++++++++")
                s_prime = enumerateState(s_prime_vals[idx], state_dimensions)
                R_val = R(s_vals, a, s_prime_vals[idx], constants)
                #print("r_val: {0}, T_prob: {1}".format(R_val, T_prob[idx]))
                new_val = T_prob[idx] * (R_val + gamma * V[s_prime])

                #if R_val <= -10000.0 and T_prob[idx] > 0.0:
                #    print("T_prob: {0}".format(T_prob))
                #    print("R_val: {0}".format(R_val))
                #    #print("new_val: {0}\n".format(new_val))
                #    print("undefined reward allowed")
                #    print("state x: {0}, y: {1}, has_food: {2}, battery: {3}, food_state: {4}".format(s_vals.x, s_vals.y, s_vals.has_food, s_vals.battery, s_vals.food_state))
                #    print("action: {0}".format(a))
                #    print("state_prime x: {0}, y: {1}, has_food: {2}, battery: {3}, food_state: {4}\n".format(s_prime_vals[idx].x, s_prime_vals[idx].y, s_prime_vals[idx].has_food, s_prime_vals[idx].battery, s_prime_vals[idx].food_state))
                #print("-------------------------------------")
                val += new_val

            #print("-----------------------")
            
            # Update best policy
            #print("val: %.6f" % val)
            #if val == 0.0:
            #    print("*****bad action: {0}".format(a))
            if val >= max_val:
                pi[s] = a # Store action with highest value
                max_val = val
        #print("max_val: %.6f" % max_val)
        V_new[s] = max_val # Update value with highest value

        # Update max difference
        max_diff = max(max_diff, abs(V[s] - V_new[s]))
    #print("V: {0}".format(V))
    #print("V_new: {0}".format(V_new))
    #diff = np.abs(np.subtract(V, V_new))
    #print("diff: {0}".format(diff))
    # Update value function
    V = np.copy(V_new)

    # If diff is smaller than threshold delta for all states, has converged, terminate
    print("max_diff: %.9f" % (max_diff))
    if max_diff < delta:
        break

# Save policy to file
print("Done!")
print(pi)
#print(V)
np.save(output_filename, pi)


#def valueIterationSingleState(s, value_lock=None, iteration_lock=None):
#    while 
#
#def poolHandler():
#    value_lock = Lock()
#    iteration_lock = Lock()
#
#    if num_parallel_threads > 1:
#        for s in range(num_states):
#            Process(target=valueIterationSingleState, args=(s, value_lock, iteration_lock)).start()
#    else:
#        for s in range(num_states):
#            valueIterationSingleState(s)
#
#if __name__ == '__main__':
#    poolHandler()
