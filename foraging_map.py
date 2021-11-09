import numpy as np

class ForagingMap:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer):
        self.food = np.array(food_layer, dtype=np.int)
        self.home = np.array(home_layer, dtype=np.int)
        self.obstacle = np.array(obstacle_layer, dtype=np.int)
        self.robot = np.array(robot_layer, dtype=np.int)
        if self.food.size() != self.home.size() and self.food.size() != self.obstacle.size() and self.food.size() != self.robot:
            raise RuntimeError("ForagaingMap: food, home, obstacle, and robot layers are not the same size\nfood:{0}, home:{1}, obstacle:{2}, robot:{3}".format(self.food.size(), self.home.size(), self.obstacle.size(), self.robot.size()))
        self.map_size = self.food.size()

    def getSubMap(self, query_x, query_y, distance):
        # Check that boundaries of queried submap do not extend outside map boundary
        # Coerce them into range if they do
        if query_x - distance < 0:
            x_min = 0
        else:
            x_min = query_x - distance

        if query_x + distance + 1 > self.map_size[0]:
            x_max = self.map_size[0]
        else:
            x_max = query_x + distance + 1

        if query_y - distance < 0:
            y_min = 0
        else:
            y_min = query_y - distance

        if query_y + distance + 1 > self.map_size[1]:
            y_max = self.map_size[1]
        else:
            y_max = query_y + distance + 1

        # Retrieve submap for each layer
        food_submap = self.food[x_min:x_max, y_min:y_max]
        home_submap = self.home[x_min:x_max, y_min:y_max]
        obstacle_submap = self.obstacle[x_min:x_max, y_min:y_max]
        robot_submap = self.robot[x_min:x_max, y_min:y_max]

        # Return tuple of submaps
        return (food_submap, home_submap, obstacle_submap, robot_submap)
