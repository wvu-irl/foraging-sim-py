import numpy as np
import matplotlib.pyplot as plt
import sys

def percentChagne(x, x_ref):
    return (x - x_ref)/abs(x_ref) * 100.0

num_args = len(sys.argv) - 1
if "--" not in sys.argv:
    raise RuntimeError("input filenames for comparison not split by '--'")    

split_index = sys.argv.index("--")

prefix1 = sys.argv[1]
prefix2 = sys.argv[split_index+1]
filename_indices = list(range(2, split_index)) + list(range(split_index+2, len(sys.argv)))
num_files = len(filename_indices)
if (num_files % 2) != 0:
    raise RuntimeError("number of input files for comparison not equal")

food_data = [None] * num_files
num_times_home_visited_data = [None] * num_files
total_distance_traversed_data = [None] * num_files
total_reward_data = [None] * num_files
battery_died = [None] * num_files

k = 0
for index in filename_indices:
    filename = sys.argv[index]
    data = np.load(filename)
    food_data[k] = data["num_food_retrieved"] #[i, j] where i = trials, j = robots
    num_times_home_visited_data[k] = data["num_times_home_visited"]
    total_distance_traversed_data[k] = data["total_distance_traversed"]
    total_reward_data[k] = data["total_reward"]
    battery_died[k] = data["battery_died"]
    k += 1

num_trials = food_data[0].shape[0]
num_robots = food_data[0].shape[1]

print("num_trials: {0}, num_robots: {1}\n".format(num_trials, num_robots))

total_food_retrieved = np.zeros((num_files, num_trials), dtype=np.int)
num_times_home_visited = np.zeros((num_files, num_trials), dtype=np.int)
total_distance_traversed = np.zeros((num_files, num_trials), dtype=np.int)
total_reward = np.zeros((num_files, num_trials), dtype=np.float)

#food_fig, food_ax = plt.subplots()
reward_fig1, reward_ax1 = plt.subplots()
reward_box_fig1, reward_box_ax1 = plt.subplots()
reward_fig2, reward_ax2 = plt.subplots()
reward_box_fig2, reward_box_ax2 = plt.subplots()
total_reward_box_data = [None] * num_files
box_plot_labels = [None] * num_files

plt.rcParams.update({"font.size": 12})
plt.rcParams.update({"font.weight": "bold"})
plt.rcParams.update({"axes.labelweight": "bold"})

max_scenario_1 = np.zeros(num_files // 2)
upper_quartile_scenario_1 = np.zeros(num_files // 2)
mean_scenario_1 = np.zeros(num_files // 2)
median_scenario_1 = np.zeros(num_files // 2)
lower_quartile_scenario_1 = np.zeros(num_files // 2)
min_scenario_1 = np.zeros(num_files // 2)
std_dev_scenario_1 = np.zeros(num_files // 2)

max_scenario_2 = np.zeros(num_files // 2)
upper_quartile_scenario_2 = np.zeros(num_files // 2)
mean_scenario_2 = np.zeros(num_files // 2)
median_scenario_2 = np.zeros(num_files // 2)
lower_quartile_scenario_2 = np.zeros(num_files // 2)
min_scenario_2 = np.zeros(num_files // 2)
std_dev_scenario_2 = np.zeros(num_files // 2)

for k in range(num_files):
    for i in range(num_trials):
        for j in range(num_robots):
            total_food_retrieved[k, i] += food_data[k][i, j]
            num_times_home_visited[k, i] += num_times_home_visited_data[k][i, j]
            total_distance_traversed[k, i] += total_distance_traversed_data[k][i, j]
            total_reward[k, i] += total_reward_data[k][i, j]

    food_mean = np.mean(total_food_retrieved[k, :])
    food_median = np.median(total_food_retrieved[k, :])
    food_max = np.max(total_food_retrieved[k, :])
    food_min = np.min(total_food_retrieved[k, :])
    food_stddev = np.std(total_food_retrieved[k, :])
    
    print("File {0} ++++++++++++++++++++++++++++++++++++++++++++++".format(k))

    print("food_mean: {0}".format(food_mean))
    print("food_median: {0}".format(food_median))
    print("food_max: {0}".format(food_max))
    print("food_min: {0}".format(food_min))
    print("food_stddev: {0}\n".format(food_stddev))

    home_visited_mean = np.mean(num_times_home_visited[k, :])
    home_visited_median = np.median(num_times_home_visited[k, :])
    home_visited_max = np.max(num_times_home_visited[k, :])
    home_visited_min = np.min(num_times_home_visited[k, :])
    home_visited_stddev = np.std(num_times_home_visited[k, :])

    print("home_visited_mean: {0}".format(home_visited_mean))
    print("home_visited_median: {0}".format(home_visited_median))
    print("home_visited_max: {0}".format(home_visited_max))
    print("home_visited_min: {0}".format(home_visited_min))
    print("home_visited_stddev: {0}\n".format(home_visited_stddev))

    distance_mean = np.mean(total_distance_traversed[k, :])
    distance_median = np.median(total_distance_traversed[k, :])
    distance_max = np.max(total_distance_traversed[k, :])
    distance_min = np.min(total_distance_traversed[k, :])
    distance_stddev = np.std(total_distance_traversed[k, :])

    print("distance_mean: {0}".format(distance_mean))
    print("distance_median: {0}".format(distance_median))
    print("distance_max: {0}".format(distance_max))
    print("distance_min: {0}".format(distance_min))
    print("distance_stddev: {0}\n".format(distance_stddev))

    reward_mean = np.mean(total_reward[k, :])
    reward_median = np.median(total_reward[k, :])
    reward_max = np.max(total_reward[k, :])
    reward_upper_quartile = np.quantile(total_reward[k, :], 0.75)
    reward_lower_quartile = np.quantile(total_reward[k, :], 0.25)
    reward_min = np.min(total_reward[k, :])
    reward_stddev = np.std(total_reward[k, :])

    if k < (num_files // 2):
        max_scenario_1[k % (num_files // 2)] = reward_max
        upper_quartile_scenario_1[k % (num_files // 2)] = reward_upper_quartile
        mean_scenario_1[k % (num_files // 2)] = reward_mean
        median_scenario_1[k % (num_files // 2)] = reward_median
        lower_quartile_scenario_1[k % (num_files // 2)] = reward_lower_quartile
        min_scenario_1[k % (num_files // 2)] = reward_min
        std_dev_scenario_1[k % (num_files // 2)] = reward_stddev
    else:
        max_scenario_2[k % (num_files // 2)] = reward_max
        upper_quartile_scenario_2[k % (num_files // 2)] = reward_upper_quartile
        mean_scenario_2[k % (num_files // 2)] = reward_mean
        median_scenario_2[k % (num_files // 2)] = reward_median
        lower_quartile_scenario_2[k % (num_files // 2)] = reward_lower_quartile
        min_scenario_2[k % (num_files // 2)] = reward_min
        std_dev_scenario_2[k % (num_files // 2)] = reward_stddev


    print("reward_max: {0}".format(reward_max))
    print("reward_upper_quartile: {0}".format(reward_upper_quartile))
    print("reward_mean: {0}".format(reward_mean))
    print("reward_median: {0}".format(reward_median))
    print("reward_lower_quartile: {0}".format(reward_lower_quartile))
    print("reward_min: {0}".format(reward_min))
    print("reward_stddev: {0}\n".format(reward_stddev))

    battery_died_percentage = float(np.count_nonzero(battery_died[k])) / (float(num_trials *num_robots)) * 100.0

    print("battery_died_percentage: {0}%".format(battery_died_percentage))

    print("---------------------------------------------------------------\n\n")

    plot_label = "$T_{" + str(k % (num_files // 2)) + "}^{t}$"
#    food_cdf_x, food_cdf_counts = np.unique(total_food_retrieved[k, :], return_counts=True)
#    food_cdf_y = np.cumsum(food_cdf_counts)
#    food_cdf_y = np.divide(food_cdf_y, food_cdf_y[-1])
#    food_cdf_x = np.insert(food_cdf_x, 0, food_cdf_x[0])
#    food_cdf_y = np.insert(food_cdf_y, 0, 0.0)
#
#    food_ax.plot(food_cdf_x, food_cdf_y, drawstyle="steps-post", label=plot_label)
#    food_ax.grid()
#    food_ax.set_xlabel("Total food retrieved")
#    food_ax.set_ylabel("Probability")
#    food_ax.legend()

    reward_cdf_x, reward_cdf_counts = np.unique(total_reward[k, :], return_counts=True)
    reward_cdf_y = np.cumsum(reward_cdf_counts)
    reward_cdf_y = np.divide(reward_cdf_y, reward_cdf_y[-1])
    reward_cdf_x = np.insert(reward_cdf_x, 0, reward_cdf_x[0])
    reward_cdf_y = np.insert(reward_cdf_y, 0, 0.0)

    if k < (num_files // 2):
        reward_ax1.plot(reward_cdf_x, reward_cdf_y, drawstyle="steps-post", label=plot_label)
        reward_ax1.grid()
        reward_ax1.set_xlabel("Total reward")
        reward_ax1.set_ylabel("Probability")
        reward_ax1.legend()
    else:
        reward_ax2.plot(reward_cdf_x, reward_cdf_y, drawstyle="steps-post", label=plot_label)
        reward_ax2.grid()
        reward_ax2.set_xlabel("Total reward")
        reward_ax2.set_ylabel("Probability")
        reward_ax2.legend()
    
    box_plot_labels[k] = plot_label
    total_reward_box_data[k] = total_reward[k, :]

per_change_max = percentChagne(max_scenario_2, max_scenario_1)
per_change_upper_quartile = percentChagne(upper_quartile_scenario_2, upper_quartile_scenario_1)
per_change_mean = percentChagne(mean_scenario_2, mean_scenario_1)
per_change_median = percentChagne(median_scenario_2, median_scenario_1)
per_change_lower_quartile = percentChagne(lower_quartile_scenario_2, lower_quartile_scenario_1)
per_change_min = percentChagne(min_scenario_2, min_scenario_1)
per_change_std_dev = percentChagne(std_dev_scenario_2, std_dev_scenario_1)

for i in range(num_files // 2):
    print("Model {0}-----------".format(i))
    print("per_change_max: %.2f" % per_change_max[i])
    print("per_change_upper_quartile: %.2f" % per_change_upper_quartile[i])
    print("per_change_mean: %.2f" % per_change_mean[i])
    print("per_change_median: %.2f" % per_change_median[i])
    print("per_change_lower_quartile: %.2f" % per_change_lower_quartile[i])
    print("per_change_min: %.2f" % per_change_min[i])
    print("per_change_std_dev: %.2f" % per_change_std_dev[i])
    print("--------------------\n")

min_limit = np.amin(total_reward) - 50
max_limit = np.amax(total_reward) + 50

reward_ax1.set_xlim(min_limit, max_limit)
reward_ax2.set_xlim(min_limit, max_limit)

reward_box_ax1.boxplot(total_reward_box_data[0:(num_files // 2)], vert = 0, notch=True, patch_artist=True)
reward_box_ax1.set_xlabel("Total Reward")
reward_box_ax1.set_yticklabels(box_plot_labels)
reward_box_ax1.grid()
reward_box_ax1.set_xlim(min_limit, max_limit)

reward_box_ax2.boxplot(total_reward_box_data[(num_files // 2):num_files], vert = 0, notch=True, patch_artist=True)
reward_box_ax2.set_xlabel("Total Reward")
reward_box_ax2.set_yticklabels(box_plot_labels)
reward_box_ax2.grid()
reward_box_ax2.set_xlim(min_limit, max_limit)

plt.show()

cdf_filename1 = "figures/" + prefix1 + "_accumulated_reward_cdf.eps"
box_filename1 = "figures/" + prefix1 + "_accumulated_reward_box_plot.eps"
reward_fig1.savefig(cdf_filename1, format="eps", bbox_inches="tight", pad_inches=0)
reward_box_fig1.savefig(box_filename1, format="eps", bbox_inches="tight", pad_inches=0)

cdf_filename2 = "figures/" + prefix2 + "_accumulated_reward_cdf.eps"
box_filename2 = "figures/" + prefix2 + "_accumulated_reward_box_plot.eps"
reward_fig2.savefig(cdf_filename2, format="eps", bbox_inches="tight", pad_inches=0)
reward_box_fig2.savefig(box_filename2, format="eps", bbox_inches="tight", pad_inches=0)
