import numpy as np
import matplotlib.pyplot as plt
import sys

def percentChange(x, x_ref):
    return (x - x_ref)/abs(x_ref) * 100.0

split_indices = [0]
for i in range(len(sys.argv)):
    if sys.argv[i] == "--":
        split_indices.append(i)
split_indices.append(len(sys.argv))

print(split_indices)
num_figures = len(split_indices) - 1
prefix = [None] * num_figures
filenames = [None] * num_figures
for i in range(num_figures):
    prefix[i] = sys.argv[split_indices[i]+1]
    filenames[i] = sys.argv[(split_indices[i]+2):(split_indices[i+1])]
    if i > 1:
        if len(filenames[i]) != len(filenames[0]):
            raise RuntimeError("number of input files for comparison not equal")

num_files_per_fig = len(filenames[0])
print("num figures: {0}".format(num_figures))
print("num_files_per_fig: {0}".format(num_files_per_fig))
print(filenames)

food_data = [[None] * num_files_per_fig] * num_figures
num_times_home_visited_data = [[None] * num_files_per_fig] * num_figures
total_distance_traversed_data = [[None] * num_files_per_fig] * num_figures
total_reward_data = [[None] * num_files_per_fig] * num_figures
battery_died = [[None] * num_files_per_fig] * num_figures

for i in range(num_figures):
    for j in range(num_files_per_fig):
        filename = filenames[i][j]
        data = np.load(filename)
        food_data[i][j] = data["num_food_retrieved"] #[k, l] where k = trials, l = robots
        num_times_home_visited_data[i][j] = data["num_times_home_visited"]
        total_distance_traversed_data[i][j] = data["total_distance_traversed"]
        total_reward_data[i][j] = data["total_reward"]
        battery_died[i][j] = data["battery_died"]

num_trials = food_data[0][0].shape[0]
num_robots = food_data[0][0].shape[1]

print("num_trials: {0}, num_robots: {1}\n".format(num_trials, num_robots))

total_food_retrieved = np.zeros((num_figures, num_files_per_fig, num_trials), dtype=np.int)
num_times_home_visited = np.zeros((num_figures, num_files_per_fig, num_trials), dtype=np.int)
total_distance_traversed = np.zeros((num_figures, num_files_per_fig, num_trials), dtype=np.int)
total_reward = np.zeros((num_figures, num_files_per_fig, num_trials), dtype=np.float)

reward_fig = [None] * num_figures
reward_ax = [None] * num_figures
reward_box_fig = [None] * num_figures
reward_box_ax = [None] * num_figures
for i in range(num_figures):
    reward_fig[i], reward_ax[i] = plt.subplots()
    reward_box_fig[i], reward_box_ax[i] = plt.subplots()
total_reward_box_data = [[None] * num_files_per_fig] * num_figures
box_plot_labels = [[None] * num_files_per_fig] * num_figures

plt.rcParams.update({"font.size": 12})
plt.rcParams.update({"font.weight": "bold"})
plt.rcParams.update({"axes.labelweight": "bold"})

max_val = np.zeros((num_figures, num_files_per_fig))
upper_quartile_val = np.zeros((num_figures, num_files_per_fig))
mean_val = np.zeros((num_figures, num_files_per_fig))
median_val = np.zeros((num_figures, num_files_per_fig))
lower_quartile_val = np.zeros((num_figures, num_files_per_fig))
min_val = np.zeros((num_figures, num_files_per_fig))
std_dev_val = np.zeros((num_figures, num_files_per_fig))

for i in range(num_figures):
    for j in range(num_files_per_fig):
        for k in range(num_trials):
            for l in range(num_robots):
                total_food_retrieved[i, j, k] += food_data[i][j][k, l]
                num_times_home_visited[i, j, k] += num_times_home_visited_data[i][j][k, l]
                total_distance_traversed[i, j, k] += total_distance_traversed_data[i][j][k, l]
                total_reward[i, j, k] += total_reward_data[i][j][k, l]

        food_mean = np.mean(total_food_retrieved[i, j, :])
        food_median = np.median(total_food_retrieved[i, j, :])
        food_max = np.max(total_food_retrieved[i, j, :])
        food_min = np.min(total_food_retrieved[i, j, :])
        food_stddev = np.std(total_food_retrieved[i, j, :])
        
        print("Fig {0}, File {1} ++++++++++++++++++++++++++++++++++++++++++++++".format(i, j))

        print("food_mean: {0}".format(food_mean))
        print("food_median: {0}".format(food_median))
        print("food_max: {0}".format(food_max))
        print("food_min: {0}".format(food_min))
        print("food_stddev: {0}\n".format(food_stddev))

        home_visited_mean = np.mean(num_times_home_visited[i, j, :])
        home_visited_median = np.median(num_times_home_visited[i, j, :])
        home_visited_max = np.max(num_times_home_visited[i, j, :])
        home_visited_min = np.min(num_times_home_visited[i, j, :])
        home_visited_stddev = np.std(num_times_home_visited[i, j, :])

        print("home_visited_mean: {0}".format(home_visited_mean))
        print("home_visited_median: {0}".format(home_visited_median))
        print("home_visited_max: {0}".format(home_visited_max))
        print("home_visited_min: {0}".format(home_visited_min))
        print("home_visited_stddev: {0}\n".format(home_visited_stddev))

        distance_mean = np.mean(total_distance_traversed[i, j, :])
        distance_median = np.median(total_distance_traversed[i, j, :])
        distance_max = np.max(total_distance_traversed[i, j, :])
        distance_min = np.min(total_distance_traversed[i, j, :])
        distance_stddev = np.std(total_distance_traversed[i, j, :])

        print("distance_mean: {0}".format(distance_mean))
        print("distance_median: {0}".format(distance_median))
        print("distance_max: {0}".format(distance_max))
        print("distance_min: {0}".format(distance_min))
        print("distance_stddev: {0}\n".format(distance_stddev))

        reward_mean = np.mean(total_reward[i, j, :])
        reward_median = np.median(total_reward[i, j, :])
        reward_max = np.max(total_reward[i, j, :])
        reward_upper_quartile = np.quantile(total_reward[i, j, :], 0.75)
        reward_lower_quartile = np.quantile(total_reward[i, j, :], 0.25)
        reward_min = np.min(total_reward[i, j, :])
        reward_stddev = np.std(total_reward[i, j, :])

        max_val[i, j] = reward_max
        upper_quartile_val[i, j] = reward_upper_quartile
        mean_val[i, j] = reward_mean
        median_val[i, j] = reward_median
        lower_quartile_val[i, j] = reward_lower_quartile
        min_val[i, j] = reward_min
        std_dev_val[i, j] = reward_stddev

        print("reward_max: {0}".format(reward_max))
        print("reward_upper_quartile: {0}".format(reward_upper_quartile))
        print("reward_mean: {0}".format(reward_mean))
        print("reward_median: {0}".format(reward_median))
        print("reward_lower_quartile: {0}".format(reward_lower_quartile))
        print("reward_min: {0}".format(reward_min))
        print("reward_stddev: {0}\n".format(reward_stddev))

        battery_died_percentage = float(np.count_nonzero(battery_died[i][j])) / (float(num_trials * num_robots)) * 100.0

        print("battery_died_percentage: {0}%".format(battery_died_percentage))

        print("---------------------------------------------------------------\n\n")

        plot_label = "$T_{" + str(j) + "}^{t}$"
    #    food_cdf_x, food_cdf_counts = np.unique(total_food_retrieved[i, j, :], return_counts=True)
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

        reward_cdf_x, reward_cdf_counts = np.unique(total_reward[i, j, :], return_counts=True)
        reward_cdf_y = np.cumsum(reward_cdf_counts)
        reward_cdf_y = np.divide(reward_cdf_y, reward_cdf_y[-1])
        reward_cdf_x = np.insert(reward_cdf_x, 0, reward_cdf_x[0])
        reward_cdf_y = np.insert(reward_cdf_y, 0, 0.0)

        reward_ax[i].plot(reward_cdf_x, reward_cdf_y, drawstyle="steps-post", label=plot_label)
        
        box_plot_labels[i][j] = plot_label
        total_reward_box_data[i][j] = total_reward[i, j, :]
       
        # Percent change, if more than one fig
        if i > 0:
            per_change_max = percentChange(max_val[i, j], max_val[0, j])
            per_change_upper_quartile = percentChange(upper_quartile_val[i, j], upper_quartile_val[0, j])
            per_change_mean = percentChange(mean_val[i, j], mean_val[0, j])
            per_change_median = percentChange(median_val[i, j], median_val[0, j])
            per_change_lower_quartile = percentChange(lower_quartile_val[i, j], lower_quartile_val[0, j])
            per_change_min = percentChange(min_val[i, j], min_val[0, j])
            per_change_std_dev = percentChange(std_dev_val[i, j], std_dev_val[0, j])

            print("Model {0}-----------".format(j))
            print("per_change_max: %.2f" % per_change_max)
            print("per_change_upper_quartile: %.2f" % per_change_upper_quartile)
            print("per_change_mean: %.2f" % per_change_mean)
            print("per_change_median: %.2f" % per_change_median)
            print("per_change_lower_quartile: %.2f" % per_change_lower_quartile)
            print("per_change_min: %.2f" % per_change_min)
            print("per_change_std_dev: %.2f" % per_change_std_dev)
            print("--------------------\n")
        
    reward_ax[i].grid()
    reward_ax[i].set_xlabel("Total reward")
    reward_ax[i].set_ylabel("Probability")
    reward_ax[i].legend()

min_limit = np.amin(total_reward) - 50
max_limit = np.amax(total_reward) + 50

for i in range(num_figures):
    reward_ax[i].set_xlim(min_limit, max_limit)

    reward_box_ax[i].boxplot(total_reward_box_data[i], vert = 0, notch=True, patch_artist=True)
    reward_box_ax[i].set_xlabel("Total Reward")
    reward_box_ax[i].set_yticklabels(box_plot_labels)
    reward_box_ax[i].grid()
    reward_box_ax[i].set_xlim(min_limit, max_limit)

    cdf_filename = "figures/" + prefix[i] + "_accumulated_reward_cdf.eps"
    box_filename = "figures/" + prefix[i] + "_accumulated_reward_box_plot.eps"
    reward_fig[i].savefig(cdf_filename, format="eps", bbox_inches="tight", pad_inches=0)
    reward_box_fig[i].savefig(box_filename, format="eps", bbox_inches="tight", pad_inches=0)

plt.show()
