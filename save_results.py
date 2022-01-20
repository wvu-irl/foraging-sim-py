import numpy as np

def saveResultsFile(filename, worlds):
    num_trials = len(worlds)
    num_robots = worlds[0].num_robots
    num_food_retrieved = np.zeros((num_trials, num_robots), dtype=np.int)
    for i in range(num_trials):
        for j in range(num_robots):
            num_food_retrieved[i, j] = worlds[i].true_robot_states[j].num_food_retrieved

    np.savez(filename,\
            num_food_retrieved = num_food_retrieved)
