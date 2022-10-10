from multiprocessing import Pool, Lock
from importlib.machinery import SourceFileLoader
from world import World
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from map_viz import displayMap
from save_results import saveResultsFile
from prev_exp import PrevExpData
import config
import sys
import time

config.enable_debug_prints = False
config.enable_plots = False
config.enable_action_policy_plots = False
save_plots = False
use_manual_control = False
slow_mode = False

config.robot_footprint_radius = 1

# Load simulation parameters
sim_params = SourceFileLoader("sim_params", sys.argv[1]).load_module()
from sim_params import *
#if sys.argv[1] == "0":
#    from params._0_single_robot_fsm_good_model import *
#elif sys.argv[1] == "1":
#    from params._1_single_robot_fsm_bad_model import *
#elif sys.argv[1] == "2":
#    from params._2_single_robot_fsm_very_bad_model import *
#elif sys.argv[1] == "3":
#    from params._3_single_robot_mdp_good_model import *
#elif sys.argv[1] == "4":
#    from params._4_single_robot_mdp_bad_model import *
#elif sys.argv[1] == "5":
#    from params._5_single_robot_mdp_very_bad_model import *
#elif sys.argv[1] == "6":
#    from params._6_single_robot_mmmdp_mv_model_0 import *
#elif sys.argv[1] == "7":
#    from params._7_single_robot_mmmdp_mv_model_1 import *
#elif sys.argv[1] == "8":
#    from params._8_single_robot_mmmdp_mv_model_2 import *
#elif sys.argv[1] == "9":
#    from params._9_single_robot_mmmdp_wmv_model_0 import *
#elif sys.argv[1] == "10":
#    from params._10_single_robot_mmmdp_wmv_model_1 import *
#elif sys.argv[1] == "11":
#    from params._11_single_robot_mmmdp_wmv_model_2 import *
#elif sys.argv[1] == "12":
#    from params._12_single_robot_mmmdp_hp_model_0 import *
#elif sys.argv[1] == "13":
#    from params._13_single_robot_mmmdp_hp_model_1 import *
#elif sys.argv[1] == "14":
#    from params._14_single_robot_mmmdp_hp_model_2 import *
#elif sys.argv[1] == "15":
#    from params._15_single_robot_mmmdp_whp_model_0 import *
#elif sys.argv[1] == "16":
#    from params._16_single_robot_mmmdp_whp_model_1 import *
#elif sys.argv[1] == "17":
#    from params._17_single_robot_mmmdp_whp_model_2 import *
#elif sys.argv[1] == "18":
#    from params._18_swarm_homo_no_local_model_0 import *
#elif sys.argv[1] == "19":
#    from params._19_swarm_homo_no_local_model_1 import *
#elif sys.argv[1] == "20":
#    from params._20_swarm_homo_no_local_model_2 import *
#elif sys.argv[1] == "21":
#    from params._21_swarm_homo_local_model_0 import *
#elif sys.argv[1] == "22":
#    from params._22_swarm_homo_local_model_1 import *
#elif sys.argv[1] == "23":
#    from params._23_swarm_homo_local_model_2 import *
#elif sys.argv[1] == "24":
#    from params._24_swarm_diverse_no_local_model_0 import *
#elif sys.argv[1] == "25":
#    from params._25_swarm_diverse_no_local_model_1 import *
#elif sys.argv[1] == "26":
#    from params._26_swarm_diverse_no_local_model_2 import *
#elif sys.argv[1] == "27":
#    from params._27_swarm_diverse_local_model_0 import *
#elif sys.argv[1] == "28":
#    from params._28_swarm_diverse_local_model_1 import *
#elif sys.argv[1] == "29":
#    from params._29_swarm_diverse_local_model_2 import *
#elif sys.argv[1] == "local":
#    from params.local_interactions_1000mc import *
#elif sys.argv[1] == "nonlocal":
#    from params.no_local_interactions_1000mc import *
#elif sys.argv[1] == "gd_train_indiv":
#    from params.groundhog_day_indiv_training_test import *
#elif sys.argv[1] == "gd_train_recur":
#    from params.groundhog_day_recur_training_test import *
#elif sys.argv[1] == "gd_eval_indiv":
#    from params.groundhog_day_indiv_eval_test import *
#elif sys.argv[1] == "gd_eval_recur":
#    from params.groundhog_day_recur_eval_test import *
#else:
#    raise RuntimeError("param file argument invalid: {0}".format(sys.argv[1]))
#from params.single_robot_fsm import *

config.enable_food_pushing = food_pushing
num_robots = len(robot_personality_list)

# Check that number of threads is less than number of Monte Carlo trials
if num_threads > num_monte_carlo_trials:
    raise RuntimeError("number of threads: {0} is greater than number of Monte Carlo trials {1}".format(num_threads, num_monte_carlo_trials))

# Load map initialization image files
food_img = Image.open(food_img_path)
food_img = food_img.transpose(Image.TRANSPOSE)
home_img = Image.open(home_img_path)
home_img = home_img.transpose(Image.TRANSPOSE)
obstacle_img = Image.open(obstacle_img_path)
obstacle_img = obstacle_img.transpose(Image.TRANSPOSE)
robot_img = Image.open(robot_img_path)
robot_img = robot_img.transpose(Image.TRANSPOSE)

# Convert images to numpy arrays
food_layer = np.array(food_img)
home_layer = np.array(home_img)
obstacle_layer = np.array(obstacle_img)
robot_layer = np.array(robot_img)

# If saving previous experience, initialize data container
if save_prev_exp:
    prev_exp_data = PrevExpData()
    prev_exp_data.allocate(num_monte_carlo_trials, num_robots, num_time_steps, list(range(num_robots)), robot_personality_list)

def runWrapper(obj): 
    # Initialize plot objects, if only one trial
    if config.enable_plots and num_threads == 1:
        map_fig, map_ax = plt.subplots()
        plt.ion()
        plt.show()

    # Run each trial
    for t in range(num_time_steps):
        # Display map for current time step, if only one trial
        if config.enable_plots and num_threads == 1:
            displayMap(obj, plt, map_fig, map_ax)
            if save_plots == 1:
                map_fig.savefig("figures/fig%d.png" % t)
            print("\nt = {0}".format(t))

        if save_prev_exp:
            prev_exp_data.record(obj, t)

        terminal_condition = obj.simulationStep(t)
        if slow_mode and num_threads == 1:
            time.sleep(0.5)
        
        if ((t == num_time_steps - 1) or (enable_terminal_condition and terminal_condition)):
            # Save final step of prev exp if at final time step
            if save_prev_exp:
                prev_exp_data.record(obj, t)

            # Display final map if at final time step
            if config.enable_plots and num_threads == 1:
                displayMap(obj, plt, map_fig, map_ax)
                if save_plots == 1:
                    t = t+1
                    map_fig.savefig("figures/fig%d.png" % t)

        # End simulation early if terminal condition reached
        if enable_terminal_condition and terminal_condition:
            break

    return obj

def poolHandler():
    # Initialize worlds
    print("Initializing worlds...")
    sim_worlds = [World(i, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, v_filepath_list, q_filepath_list, arbitration_type_list, use_prev_exp, prev_exp_filepath, num_time_steps, heading_change_times, food_respawn, real_world_exp=False, manual_control=use_manual_control) for i in range(num_monte_carlo_trials)]
    
    # Run pool of Monte Carlo trials
    print("Beginning Monte Carlo trials...")
    if num_threads > 1 and not (save_prev_exp and recursive_prev_exp):
        # Initialize multiprocessing pool and map objects' run methods to begin simulation
        p = Pool(num_threads)
        sim_worlds = p.map(runWrapper, sim_worlds)
        if save_prev_exp:
            prev_exp_data.last_trial_written[0] = num_monte_carlo_trials - 1
    else:
        # Run each trial sequentially, for debugging purposes or for groundhog day recursive training, with no multiprocessing
        for i in range(num_monte_carlo_trials):
            runWrapper(sim_worlds[i])
            if save_prev_exp:
                prev_exp_data.last_trial_written[0] = i
                if recursive_prev_exp:
                    prev_exp_data.save(prev_exp_filepath)
    print("Simulations complete.")

    # Save results
    saveResultsFile(results_filename, sim_worlds)
    if save_prev_exp:
        prev_exp_data.save(prev_exp_filepath)

if __name__=='__main__':
    poolHandler()
