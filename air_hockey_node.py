#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
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
config.enable_plots = True
config.enable_action_policy_plots = False
save_plots = False
use_manual_control = False
slow_mode = False

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
#else:
#    raise RuntimeError("param file argument invalid: {0}".format(sys.argv[1]))

config.enable_food_pushing = food_pushing
num_robots = len(robot_personality_list)

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

def runWrapper(obj, map_fig, map_ax):
    plt.ion()
    plt.show()
    displayMap(obj, plt, map_fig, map_ax)

    for t in range(num_time_steps):
        if rospy.is_shutdown():
            break

        displayMap(obj, plt, map_fig, map_ax) 
        if save_plots == 1:
            map_fig.savefig("figures/fig%d.png" % t)
        print("t = {0}".format(t))

        terminal_condition = obj.simulationStep()
        if slow_mode:
            time.sleep(0.5)

        if ((t == num_time_steps - 1) or (enable_terminal_condition and terminal_condition)):
            # Save final step of prev exp if at final time step
            if save_prev_exp: prev_exp_data.record(obj, t)

            # Display final map if at final time step
            displayMap(obj, plt, map_fig, map_ax) 
            if save_plots == 1:
                t = t+1
                map_fig.savefig("figures/fig%d.png" % t)


def run():
    rospy.init_node("foraging_air_hockey_interface")
    enable_waypoint_pub = rospy.Publisher("turtle1/use_waypoint", Bool, queue_size=1, latch=True) # TODO: need this topic name
    enable_waypoint_msg = Bool()
    worlds = [World(i, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, v_filepath_list, q_filepath_list, arbitration_type_list, use_prev_exp, prev_exp_filepath, num_time_steps, heading_change_times, food_respawn, real_world_exp=True, manual_control=use_manual_control) for i in range(num_monte_carlo_trials)]
    plt.switch_backend("QT4Agg")
    mgr = plt.get_current_fig_manager()
    #mgr.full_screen_toggle()
    py = mgr.canvas.height()
    px = mgr.canvas.width()
    mgr.window.close()
    map_fig, map_ax = plt.subplots()
    mgr = plt.get_current_fig_manager()
    #mgr.window.move(0, py)
    #mgr.full_screen_toggle()
    for i in range(num_monte_carlo_trials):
        enable_waypoint_msg.data = True
        enable_waypoint_pub.publish(enable_waypoint_msg)
        runWrapper(worlds[i], map_fig, map_ax)
        enable_waypoint_msg.data = False
        enable_waypoint_pub.publish(enable_waypoint_msg)
        if rospy.is_shutdown():
            break
        if save_prev_exp:
            prev_exp_data.last_trial_written[0] = i
            if recursive_prev_exp:
                prev_exp_data.save(prev_exp_filepath)
        if (i + 1) < (num_monte_carlo_trials - 1):
            init_pos = worlds[i+1].true_constants[0]["init_pos"]
            goal_msg = Twist()
            goal_msg.linear.x = init_pos[0] * self.grid_to_vicon_conv_factor
            goal_msg.linear.y = init_pos[1] * self.grid_to_vicon_conv_factor
            worlds[i+1].real_world_interface[0].waypoint_pub.publish(goal_msg) 
            displayMap(worlds[i+1], plt, map_fig, map_ax)
            input("Please reset world as shown and press enter to begin next trial")

    # Save results
    saveResultsFile(results_filename, worlds)
    if save_prev_exp:
        prev_exp_data.save(prev_exp_filepath)

if __name__=='__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
