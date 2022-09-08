food_img_path = "maps/5_5_food_map.png"
home_img_path = "maps/5_5_home_map.png"
obstacle_img_path = "maps/5_5_obstacle_map.png"
robot_img_path = "maps/5_5_robot_map.png"
robot_personality_list = [10]
perception_range = 1
battery_size = 10
heading_size = 8
policy_filepath_list = [["policies/single_robot_vi_policy_0.npy", "policies/single_robot_vi_policy_1.npy", "policies/single_robot_vi_policy_2.npy", "policies/single_robot_vi_policy_3.npy", "policies/single_robot_vi_policy_4.npy"]]
v_filepath_list = [["policies/single_robot_vi_state_value_0.npy", "policies/single_robot_vi_state_value_1.npy", "policies/single_robot_vi_state_value_2.npy", "policies/single_robot_vi_state_value_3.npy", "policies/single_robot_vi_state_value_4.npy"]]
q_filepath_list = [["policies/single_robot_vi_state_action_value_0.npy", "policies/single_robot_vi_state_action_value_1.npy", "policies/single_robot_vi_state_action_value_2.npy", "policies/single_robot_vi_state_action_value_3.npy", "policies/single_robot_vi_state_action_value_4.npy"]]
arbitration_type_list = [0]
num_time_steps = 100
heading_change_times = []
num_monte_carlo_trials = 1
num_threads = 1
food_respawn = True
food_pushing = False
results_filename = "results/test_6_single_robot_mmmdp_mv_model_0_results.npz"
