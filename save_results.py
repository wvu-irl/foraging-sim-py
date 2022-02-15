import numpy as np

def saveResultsFile(filename, worlds):
    num_trials = len(worlds)
    num_robots = worlds[0].num_robots
    num_food_retrieved = np.zeros((num_trials, num_robots), dtype=np.int)
    num_times_home_visited = np.zeros((num_trials, num_robots), dtype=np.int)
    total_distance_traversed = np.zeros((num_trials, num_robots), dtype=np.int)
    total_reward = np.zeros((num_trials, num_robots), dtype=np.float)
    for i in range(num_trials):
        for j in range(num_robots):
            num_food_retrieved[i, j] = worlds[i].true_robot_states[j].num_food_retrieved
            num_times_home_visited[i, j] = worlds[i].true_robot_states[j].num_times_home_visited
            total_distance_traversed[i, j] = worlds[i].true_robot_states[j].total_distance_traversed
            total_reward[i, j] = worlds[i].true_total_reward[j]

    np.savez(filename,\
            num_food_retrieved = num_food_retrieved,\
            num_times_home_visited = num_times_home_visited,\
            total_distance_traversed = total_distance_traversed,\
            total_reward = total_reward)
