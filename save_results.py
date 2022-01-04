import numpy as np

def saveResultsFile(filename, world):
    num_food_retrieved = np.zeros(world.num_robots, dtype=np.int)
    for i in range(world.num_robots):
        num_food_retrieved[i] = world.true_robot_states[i].num_food_retrieved

    np.savez(filename,\
            num_food_retrieved = num_food_retrieved)
