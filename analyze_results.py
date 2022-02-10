import numpy as np
import sys

filename = sys.argv[1] 

data = np.load(filename)
food_data = data["num_food_retrieved"] #[i, j] where i = trials, j = robots

num_trials = food_data.shape[0]
num_robots = food_data.shape[1]

total_food_retrieved = np.zeros(num_trials, dtype=np.int)
for i in range(num_trials):
    for j in range(num_robots):
        total_food_retrieved[i] += food_data[i, j]

food_mean = np.mean(total_food_retrieved)
food_max = np.max(total_food_retrieved)
food_min = np.min(total_food_retrieved)
food_stddev = np.std(total_food_retrieved)

print("food_mean: {0}".format(food_mean))
print("food_max: {0}".format(food_max))
print("food_min: {0}".format(food_min))
print("food_stddev: {0}".format(food_stddev))
