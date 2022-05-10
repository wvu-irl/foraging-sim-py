#!/usr/bin/env python3
import rospy
from world import World
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
from map_viz import displayMap
from save_results import saveResultsFile
import config
import sys

config.enable_debug_prints = False
save_plots = False

# Load simulation parameters
if sys.argv[1] == "0":
    from params._0_single_robot_fsm_good_model import *
elif sys.argv[1] == "1":
    from params._1_single_robot_fsm_bad_model import *
elif sys.argv[1] == "2":
    from params._2_single_robot_fsm_very_bad_model import *
elif sys.argv[1] == "3":
    from params._3_single_robot_mdp_good_model import *
elif sys.argv[1] == "4":
    from params._4_single_robot_mdp_bad_model import *
elif sys.argv[1] == "5":
    from params._5_single_robot_mdp_very_bad_model import *
elif sys.argv[1] == "6":
    from params._6_single_robot_mmmdp_mv_model_0 import *
elif sys.argv[1] == "7":
    from params._7_single_robot_mmmdp_mv_model_1 import *
elif sys.argv[1] == "8":
    from params._8_single_robot_mmmdp_mv_model_2 import *
elif sys.argv[1] == "9":
    from params._9_single_robot_mmmdp_wmv_model_0 import *
elif sys.argv[1] == "10":
    from params._10_single_robot_mmmdp_wmv_model_1 import *
elif sys.argv[1] == "11":
    from params._11_single_robot_mmmdp_wmv_model_2 import *
elif sys.argv[1] == "12":
    from params._12_single_robot_mmmdp_hp_model_0 import *
elif sys.argv[1] == "13":
    from params._13_single_robot_mmmdp_hp_model_1 import *
elif sys.argv[1] == "14":
    from params._14_single_robot_mmmdp_hp_model_2 import *
elif sys.argv[1] == "15":
    from params._15_single_robot_mmmdp_whp_model_0 import *
elif sys.argv[1] == "16":
    from params._16_single_robot_mmmdp_whp_model_1 import *
elif sys.argv[1] == "17":
    from params._17_single_robot_mmmdp_whp_model_2 import *
elif sys.argv[1] == "18":
    from params._18_swarm_homo_no_local_model_0 import *
elif sys.argv[1] == "19":
    from params._19_swarm_homo_no_local_model_1 import *
elif sys.argv[1] == "20":
    from params._20_swarm_homo_no_local_model_2 import *
elif sys.argv[1] == "21":
    from params._21_swarm_homo_local_model_0 import *
elif sys.argv[1] == "22":
    from params._22_swarm_homo_local_model_1 import *
elif sys.argv[1] == "23":
    from params._23_swarm_homo_local_model_2 import *
elif sys.argv[1] == "24":
    from params._24_swarm_diverse_no_local_model_0 import *
elif sys.argv[1] == "25":
    from params._25_swarm_diverse_no_local_model_1 import *
elif sys.argv[1] == "26":
    from params._26_swarm_diverse_no_local_model_2 import *
elif sys.argv[1] == "27":
    from params._27_swarm_diverse_local_model_0 import *
elif sys.argv[1] == "28":
    from params._28_swarm_diverse_local_model_1 import *
elif sys.argv[1] == "29":
    from params._29_swarm_diverse_local_model_2 import *
else:
    raise RuntimeError("param file argument invalid: {0}".format(sys.argv[1]))

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

def run():
    rospy.init_node("foraging_air_hockey_interface")
    world = World(food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, v_filepath_list, q_filepath_list, arbitration_type_list, num_time_steps, real_world_exp=True)
    map_fig, map_ax = plt.subplots()
    plt.ion()
    plt.show()

    for t in range(num_time_steps):
        world.simulationStep()
        displayMap(world, plt, map_fig, map_ax) 
        if save_plots == 1:
            map_fig.savefig("figures/fig%d.png" % t)
        print("t = {0}".format(t))
        if rospy.is_shutdown():
            break

    # Save results
    saveResultsFile(results_filename, [world])

if __name__=='__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
