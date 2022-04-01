from multiprocessing import Pool
from world import World
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from map_viz import displayMap
from save_results import saveResultsFile
import config
import sys

config.enable_debug_prints = False
enable_plots = False
save_plots = False

# Load simulation parameters
if sys.argv[1] == "0":
    from params._0_single_robot_fsm import *
elif sys.argv[1] == "1":
    from params._1_single_robot_mdp_correct_model import *
elif sys.argv[1] == "2":
    from params._2_single_robot_mdp_wrong_model import *
else:
    raise RuntimeError("param file argument invalid: {0}".format(sys.argv[1]))
#from params.single_robot_fsm import *

# Check that number of threads is less than number of Monte Carlo trials
if num_threads > num_monte_carlo_trials:
    raise RuntimeError("number of threads: {0} is greater than number of Monte Carlo trials {1}".format(num_threads, num_monte_carlo_trials))

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

def runWrapper(obj): 
    # Initialize plot objects, if only one trial
    if enable_plots and num_threads == 1:
        map_fig, map_ax = plt.subplots()
        plt.ion()
        plt.show()

    # Run each trial
    for t in range(num_time_steps):
        obj.simulationStep()

        # Display map for current time step, if only one trial
        if enable_plots and num_threads == 1:
            displayMap(obj, plt, map_fig, map_ax)
            if save_plots == 1:
                map_fig.savefig("figures/fig%02d.png" % t)
            print("t = {0}".format(t))

    return obj

def poolHandler():
    # Initialize worlds
    print("Initializing worlds...")
    sim_worlds = [World(food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, num_time_steps) for i in range(num_monte_carlo_trials)]
    
    # Run pool of Monte Carlo trials
    print("Beginning Monte Carlo trials...")
    if num_threads > 1:
        # Initialize multiprocessing pool and map objects' run methods to begin simulation
        p = Pool(num_threads)
        sim_worlds = p.map(runWrapper, sim_worlds)
    else:
        # Run each trial sequentially, for debugging purposes, with no multiprocessing
        for i in range(num_monte_carlo_trials):
            runWrapper(sim_worlds[i])
    print("Simulations complete.")

    # Save results
    saveResultsFile(results_filename, sim_worlds)

if __name__=='__main__':
    poolHandler()
