import numpy as np

class ForagaingMap:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer):
        self.food = np.array(food_layer, dtype=np.int)
        self.home = np.array(home_layer, dtype=np.int)
        self.obstacle = np.array(obstacle_layer, dtype=np.int)
        self.robot = np.array(robot_layer, dtype=np.int)
        if self.food.size() != self.home.size() and self.food.size() != self.obstacle.size() and self.food.size() != self.robot:
            raise RuntimeError("ForagaingMap: food, home, obstacle, and robot layers are not the same size\nfood:{0}, home:{1}, obstacle:{2}, robot:{3}".format(self.food.size(), self.home.size(), self.obstacle.size(), self.robot.size()))
