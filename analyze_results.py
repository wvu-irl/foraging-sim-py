import numpy as np
import matplotlib.pyplot as plt
import sys

filename = sys.argv[1] 

data = np.load(filename)
food_data = data["num_food_retrieved"] #[i, j] where i = trials, j = robots
num_times_home_visited_data = data["num_times_home_visited"]
total_distance_traversed_data = data["total_distance_traversed"]
total_reward_data = data["total_reward"]
battery_died = data["battery_died"]

num_trials = food_data.shape[0]
num_robots = food_data.shape[1]

print("num_trials: {0}, num_robots: {1}\n".format(num_trials, num_robots))

total_food_retrieved = np.zeros(num_trials, dtype=np.int)
num_times_home_visited = np.zeros(num_trials, dtype=np.int)
total_distance_traversed = np.zeros(num_trials, dtype=np.int)
total_reward = np.zeros(num_trials, dtype=np.int)
for i in range(num_trials):
    for j in range(num_robots):
        total_food_retrieved[i] += food_data[i, j]
        num_times_home_visited[i] += num_times_home_visited_data[i, j]
        total_distance_traversed[i] += total_distance_traversed_data[i, j]
        total_reward[i] += total_reward_data[i, j]

food_mean = np.mean(total_food_retrieved)
food_max = np.max(total_food_retrieved)
food_min = np.min(total_food_retrieved)
food_stddev = np.std(total_food_retrieved)

print("food_mean: {0}".format(food_mean))
print("food_max: {0}".format(food_max))
print("food_min: {0}".format(food_min))
print("food_stddev: {0}\n".format(food_stddev))

home_visited_mean = np.mean(num_times_home_visited)
home_visited_max = np.max(num_times_home_visited)
home_visited_min = np.min(num_times_home_visited)
home_visited_stddev = np.std(num_times_home_visited)

print("home_visited_mean: {0}".format(home_visited_mean))
print("home_visited_max: {0}".format(home_visited_max))
print("home_visited_min: {0}".format(home_visited_min))
print("home_visited_stddev: {0}\n".format(home_visited_stddev))

distance_mean = np.mean(total_distance_traversed)
distance_max = np.max(total_distance_traversed)
distance_min = np.min(total_distance_traversed)
distance_stddev = np.std(total_distance_traversed)

print("distance_mean: {0}".format(distance_mean))
print("distance_max: {0}".format(distance_max))
print("distance_min: {0}".format(distance_min))
print("distance_stddev: {0}\n".format(distance_stddev))

reward_mean = np.mean(total_reward)
reward_max = np.max(total_reward)
reward_min = np.min(total_reward)
reward_stddev = np.std(total_reward)

print("reward_mean: {0}".format(reward_mean))
print("reward_max: {0}".format(reward_max))
print("reward_min: {0}".format(reward_min))
print("reward_stddev: {0}\n".format(reward_stddev))

battery_died_percentage = float(np.count_nonzero(battery_died)) / (float(num_trials *num_robots)) * 100.0

print("battery_died_percentage: {0}%".format(battery_died_percentage))

food_cdf_x, food_cdf_counts = np.unique(total_food_retrieved, return_counts=True)
food_cdf_y = np.cumsum(food_cdf_counts)
food_cdf_y = np.divide(food_cdf_y, food_cdf_y[-1])
food_cdf_x = np.insert(food_cdf_x, 0, food_cdf_x[0])
food_cdf_y = np.insert(food_cdf_y, 0, 0.0)

food_fig, food_ax = plt.subplots()
food_ax.plot(food_cdf_x, food_cdf_y, drawstyle="steps-post")
food_ax.grid()
food_ax.set_xlabel("Total food retrieved")
food_ax.set_ylabel("Probability")

reward_cdf_x, reward_cdf_counts = np.unique(total_reward, return_counts=True)
reward_cdf_y = np.cumsum(reward_cdf_counts)
reward_cdf_y = np.divide(reward_cdf_y, reward_cdf_y[-1])
reward_cdf_x = np.insert(reward_cdf_x, 0, reward_cdf_x[0])
reward_cdf_y = np.insert(reward_cdf_y, 0, 0.0)

reward_fig, reward_ax = plt.subplots()
reward_ax.plot(reward_cdf_x, reward_cdf_y, drawstyle="steps-post")
reward_ax.grid()
reward_ax.set_xlabel("Total reward")
reward_ax.set_ylabel("Probability")

plt.show()
