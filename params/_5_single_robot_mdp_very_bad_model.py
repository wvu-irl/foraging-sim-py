food_img_path = "maps/5_5_food_different_headings2_map.png"
home_img_path = "maps/5_5_home_map.png"
obstacle_img_path = "maps/5_5_obstacle_map.png"
robot_img_path = "maps/5_5_robot_map.png"
robot_personality_list = [7]
perception_range = 1
battery_size = 10
heading_size = 8
policy_filepath_list = ["policies/single_robot_vi_policy_0.npy"]
v_filepath_list = ["policies/single_robot_vi_state_value_0.npy"]
q_filepath_list = ["policies/single_robot_vi_state_action_value_0.npy"]
arbitration_type_list = []
num_time_steps = 100
num_monte_carlo_trials = 1000
num_threads = 80
results_filename = "results/5_single_robot_mdp_very_bad_model_results.npz"
