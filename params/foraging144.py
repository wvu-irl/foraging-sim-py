food_img_path = "maps/25_25_food_map_d.png"
home_img_path = "maps/25_25_home_map_d.png"
obstacle_img_path = "maps/25_25_obstacle_map.png"
robot_img_path = "maps/25_25_robot_map_d_12_bots.png"
robot_personality_list = [13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14]
perception_range = 2
battery_size = 50
heading_size = 8
policy_filepath_list = []
v_filepath_list = []
q_filepath_list = []
arbitration_type_list = []
use_prev_exp = False
save_prev_exp = False
recursive_prev_exp = False
prev_exp_filepath = ""
num_time_steps = 300
heading_change_times = []
num_monte_carlo_trials = 1000
num_threads = 80
food_respawn = False
food_pushing = True
enable_terminal_condition = False
results_filename = "results/foraging144.npz"
plot_label = "Model 3"
human_description = "25x25, map d, 12 bots, per range 2, no local, diverse"
