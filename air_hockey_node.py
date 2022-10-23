#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import Twist
from importlib.machinery import SourceFileLoader
from world import World
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy import interpolate
from map_viz import displayMap
from save_results import saveResultsFile
from prev_exp import PrevExpData
import config
import sys
import time

config.enable_debug_prints = True
config.enable_plots = True
config.enable_action_policy_plots = False
config.use_bug_avoidance = True
save_plots = False
use_manual_control = False
slow_mode = False

config.robot_footprint_radius = 3

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
map_shape = np.shape(food_layer)
projector_dpi = 16 # pixels per inch
projector_width = 1650
projector_height = 825

# ROS image and publisher for publishing map to image server
image_msg = ImageMsg()
image_msg.header.seq = 0
image_msg.width = projector_width
image_msg.height = projector_height
image_msg.encoding = "bgr8"
image_msg.is_bigendian = 0
image_msg.step = 3 * image_msg.width
image_msg.data = [0] * image_msg.height * image_msg.step
image_pub = rospy.Publisher("foraging_map_img", ImageMsg, queue_size=1, latch=True)

# If saving previous experience, initialize data container
if save_prev_exp:
    prev_exp_data = PrevExpData()
    prev_exp_data.allocate(num_monte_carlo_trials, num_robots, num_time_steps, list(range(num_robots)), robot_personality_list)
    if recursive_prev_exp:
        prev_exp_data.save(prev_exp_filepath)

def pubMatplotlibImage(fig, ax):
    #img_x = np.linspace(0, 1, img.shape[0])
    #img_y = np.linspace(0, 1, img.shape[1])
    #interp_obj_r = interpolate.interp2d(img_x, img_y, img[:, :, 0], kind="cubic")
    #interp_obj_g = interpolate.interp2d(img_x, img_y, img[:, :, 1], kind="cubic")
    #interp_obj_b = interpolate.interp2d(img_x, img_y, img[:, :, 2], kind="cubic")
    #upscaled_x = np.linspace(0, 1, 1920)
    #upscaled_y = np.linspace(0, 1, 1080)
    #upscaled_img_r = interp_obj_r(upscaled_x, upscaled_y)
    #upscaled_img_g = interp_obj_g(upscaled_x, upscaled_y)
    #upscaled_img_b = interp_obj_b(upscaled_x, upscaled_y)
    #upscaled_img = np.zeros((1920, 1080, 3), dtype=np.uint8)
    #upscaled_img[:, :, 0] = upscaled_img_r
    #upscaled_img[:, :, 1] = upscaled_img_g
    #upscaled_img[:, :, 2] = upscaled_img_b
    #upscaled_img = cv2.resize(np.swapaxes(img, 0, 1), dsize=(1920, 1080), interpolation=cv2.INTER_CUBIC)
    fig.canvas.draw()
    scaled_img = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()), cv2.COLOR_RGBA2BGR)
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(scaled_img, encoding="passthrough")
    image_msg.encoding = "bgr8"
    #print("upscaled_img shape: [{0}, {1}]".format(upscaled_img.shape[0], upscaled_img.shape[1]))
    image_msg.header.stamp = rospy.get_rostime()
    #i = 0
    #for y in range(upscaled_img.shape[1]): # Image height
    #    for x in range(upscaled_img.shape[0]): # Image width
    #        for z in range(3): # R, G, B
    #            image_msg.data[i] = upscaled_img[x, map_shape[1]-1 - y, z]
    #            i += 1
    image_pub.publish(image_msg)
    image_msg.header.seq += 1

def runWrapper(obj, map_fig, map_ax):
    for t in range(num_time_steps):
        if rospy.is_shutdown():
            break

        if save_prev_exp:
             prev_exp_data.record(obj, t)

        terminal_condition = obj.simulationStep(t)
        if slow_mode:
            time.sleep(0.5)
        
        map_fig.set_figwidth(projector_width / projector_dpi)
        map_fig.set_figheight(projector_height / projector_dpi)
        #map_fig.set_dpi(projector_dpi)
        np_img = displayMap(obj, plt, map_fig, map_ax) 
        pubMatplotlibImage(map_fig, map_ax) 
        if save_plots == 1:
            map_fig.savefig("figures/fig%d.png" % t)
        print("t = {0}".format(t))

        if ((t == num_time_steps - 1) or (enable_terminal_condition and terminal_condition)):
            # Save final step of prev exp if at final time step
            if save_prev_exp:
                 prev_exp_data.record(obj, t)

            # Display final map if at final time step
            map_fig.set_figwidth(projector_width / projector_dpi)
            map_fig.set_figheight(projector_height / projector_dpi)
            #map_fig.set_dpi(projector_dpi)
            np_img = displayMap(obj, plt, map_fig, map_ax) 
            pubMatplotlibImage(map_fig, map_ax)
            if save_plots == 1:
                t = t+1
                map_fig.savefig("figures/fig%d.png" % t)

def run():
    rospy.init_node("foraging_air_hockey_interface")
    worlds = [World(i, food_layer, home_layer, obstacle_layer, robot_layer, robot_personality_list, perception_range, battery_size, heading_size, policy_filepath_list, v_filepath_list, q_filepath_list, arbitration_type_list, use_prev_exp, prev_exp_filepath, num_time_steps, heading_change_times, food_respawn, real_world_exp=True, manual_control=use_manual_control) for i in range(num_monte_carlo_trials)]
    plt.switch_backend("QT4Agg") # TODO: change this to publish map as img to ROS web_video_server
    #mgr = plt.get_current_fig_manager()
    #mgr.full_screen_toggle()
    #py = mgr.canvas.height()
    #px = mgr.canvas.width()
    #mgr.window.close()
    plt.ioff()
    map_fig, map_ax = plt.subplots(frameon=False)
    map_ax.margins(0)
    map_ax.set_position([0, 0, 1, 1])
    #map_ax.set_aspect("equal")
    #map_ax.xaxis.set_visible(False)
    #map_ax.yaxis.set_visible(False)
    #for spine in ["top", "right", "left", "bottom"]:
    #    map_ax.spines[spine].set_visible(False)
    #map_fig.tight_layout()
    #mgr = plt.get_current_fig_manager()
    #mgr.window.move(0, py)
    #mgr.full_screen_toggle()
    map_fig.set_figwidth(projector_width / projector_dpi)
    map_fig.set_figheight(projector_height / projector_dpi)
    map_fig.set_dpi(projector_dpi)
    #plt.ion()
    #plt.show()
    init_pos = worlds[0].true_constants[0]["init_pos"]
    worlds[0].real_world_interface[0].sendInitCmd(init_pos[0], init_pos[1])
    np_img = displayMap(worlds[0], plt, map_fig, map_ax)
    pubMatplotlibImage(map_fig, map_ax)
    input("Please set world as shown and then press enter to begin")
    for i in range(num_monte_carlo_trials):
        if recursive_prev_exp:
            worlds[i].initPrevExp()
        runWrapper(worlds[i], map_fig, map_ax)
        if rospy.is_shutdown():
            break
        if save_prev_exp:
            prev_exp_data.last_trial_written[0] = i
            if recursive_prev_exp:
                prev_exp_data.save(prev_exp_filepath)
        if i < (num_monte_carlo_trials - 1): 
            init_pos = worlds[i+1].true_constants[0]["init_pos"]
            worlds[i+1].real_world_interface[0].sendInitCmd(init_pos[0], init_pos[1])
            map_fig.set_figwidth(projector_width / projector_dpi)
            map_fig.set_figheight(projector_height / projector_dpi)
            #map_fig.set_dpi(projector_dpi)
            np_img = displayMap(worlds[i+1], plt, map_fig, map_ax)
            pubMatplotlibImage(map_fig, map_ax)
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
