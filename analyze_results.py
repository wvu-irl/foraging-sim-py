import numpy as np
import sys

filename = sys.argv[1] 

data = np.load(filename)
food_data = data["num_food_retrieved"] #[i, j] where i = trials, j = robots
num_times_home_visited_data = data["num_times_home_visited"]
total_distance_traversed_data = data["total_distance_traversed"]
total_reward_data = data["total_reward"]

num_trials = food_data.shape[0]
num_robots = food_data.shape[1]

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
print("reward_stddev: {0}".format(reward_stddev))
