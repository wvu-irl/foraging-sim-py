from world import World
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from map_viz import displayMap
from save_results import saveResultsFile
import config

config.enable_debug_prints = False

# TODO: modify to run multiple copies of sim in parallel for Monte Carlo trials

# Load simulation parameters
from params.scenario_2_params import *

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

# Initialize world
sim_world = World(food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range)

# Initialize plot objects
map_fig, map_ax = plt.subplots()
plt.ion()
plt.show()

# Run simulation for prescribed number of timesteps
for t in range(num_time_steps):
    sim_world.simulationStep()

    # Display map for current time step
    displayMap(sim_world.map, plt, map_fig, map_ax)
    print("t = {0}".format(t))

# Save results
saveResultsFile(results_filename, sim_world)
