import numpy as np
from states import *
from actions import *
from transition_models import *
from reward_functions import *

from params.scenario_1_params import *

# Load map initialization image files
food_img = Image.open(food_img_path)

# Convert images to numpy arrays
food_layer = np.array(food_img)
home_img = Image.open(home_img_path)
obstacle_img = Image.open(obstacle_img_path)
robot_img = Image.open(robot_img_path)

# Initialize map
map_obj = ForagingMap(food_layer, home_layer, obstacle_layer, robot_layer)
map_shape = map_obj.map_shape
        
# Record the food positions and headings lists
food_pos, food_heading = map_obj.findFoodInfo()
num_food = len(food_pos)
        
# Record home location
home_pos = map_obj.findHomePosition(0)

# Record constants and state dimensions
constants = {"map_shape" : map_shape, "battery_size" : battery_size, "home_pos" : home_pos,"num_food" : num_food, "food_pos" : food_pos}
state_dimensions = {"x_size" : map_shape[0], "y_size" : map_shape[1], "has_food_size" : 2, "battery_size" : battery_size, "num_food" : num_food}

# Set value iteration parameters
max_iter = 10000  # Maximum number of iterations
delta = 1e-400  # Error tolerance


