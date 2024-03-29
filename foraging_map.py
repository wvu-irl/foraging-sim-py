import numpy as np
from enum import IntEnum, unique
import sys

@unique
class MapLayer(IntEnum):
    NUM_LAYERS = 4

    FOOD = 0
    HOME = 1
    OBSTACLE = 2
    ROBOT = 3

class ForagingMap:
    def __init__(self, food_layer, home_layer, obstacle_layer, robot_layer):
        if food_layer.shape != home_layer.shape and food_layer.shape != obstacle_layer.shape and food_layer.shape != robot_layer.shape:
            raise RuntimeError("ForagaingMap: food, home, obstacle, and robot layers are not the same shape\nfood:{0}, home:{1}, obstacle:{2}, robot:{3}".format(food_layer.shape, home_layer.shape, obstacle_layer.shape, robot_layer.shape))
        self.map_shape = food_layer.shape
        map_obj_shape = (MapLayer.NUM_LAYERS,) + self.map_shape
        self.map = np.zeros(map_obj_shape, dtype=np.int)
        self.map[MapLayer.FOOD] = food_layer
        self.map[MapLayer.HOME] = home_layer
        self.map[MapLayer.OBSTACLE] = obstacle_layer
        self.map[MapLayer.ROBOT] = robot_layer

    def getSubMap(self, query_x, query_y, distance, true_states, constants):
        #if query_x - distance < 0:
        #    x_min = 0
        #else:
        x_min = query_x - distance

        #if query_x + distance + 1 > self.map_shape[0]:
        #    x_max = self.map_shape[0]
        #else:
        x_max = query_x + distance + 1

        #if query_y - distance < 0:
        #    y_min = 0
        #else:
        y_min = query_y - distance

        #if query_y + distance + 1 > self.map_shape[1]:
        #    y_max = self.map_shape[1]
        #else:
        y_max = query_y + distance + 1

        # Retrieve submap
        submap = self.map[:, x_min:x_max, y_min:y_max]

        # Iterate over submap and populate lists of neighboring objects
        submap_object_list = []
        submap_property_list = []
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                # Find relative position of grid cell
                delta_x = int(x - query_x)
                delta_y = int(y - query_y)

                # If x,y outside map boundary, list it as an obstacle
                if (x not in range(0, self.map_shape[0])) or (y not in range(0, self.map_shape[1])):
                    submap_object_list.append(MapLayer.OBSTACLE)
                    submap_property_list.append({"delta_x" : delta_x, "delta_y" : delta_y})
                else:
                   
                    # Check food layer
                    if self.map[MapLayer.FOOD, x, y] > 0: # Contains food
                        submap_object_list.append(MapLayer.FOOD)
                        submap_property_list.append({"delta_x" : delta_x, "delta_y" : delta_y, "heading" : self.map[MapLayer.FOOD, x, y]})

                    # Check home layer
                    if self.map[MapLayer.HOME, x, y] == 1: # Is a home cell
                        submap_object_list.append(MapLayer.HOME)
                        submap_property_list.append({"delta_x" : delta_x, "delta_y" : delta_y})

                    # Check obstacle layer
                    if self.map[MapLayer.OBSTACLE, x, y] == 1: # Contains static obstacle
                        submap_object_list.append(MapLayer.OBSTACLE)
                        submap_property_list.append({"delta_x" : delta_x, "delta_y" : delta_y})

                    # Check robot layer
                    if self.map[MapLayer.ROBOT, x, y] != 0 and (delta_x != 0 or delta_y != 0): # Contains another robot
                        robot_id = self.map[MapLayer.ROBOT, x, y] - 1
                        submap_object_list.append(MapLayer.ROBOT)
                        robot_properties_dict = {**{"delta_x" : delta_x, "delta_y" : delta_y, "id" : robot_id, "personality" : constants[robot_id]["personality"]}, **true_states[robot_id].localInfluenceData()}
                        submap_property_list.append(robot_properties_dict)
                        # Other robot is also an obstacle if not a phantom
                        if constants[robot_id]["phantom"] == False:
                            submap_object_list.append(MapLayer.OBSTACLE)
                            submap_property_list.append({"delta_x" : delta_x, "delta_y" : delta_y})

        # Return object type and property lists
        return (submap_object_list, submap_property_list)


    def setSubMap(self, center_x, center_y, submap):
        # Loop over entries in submap list
        submap_object_list = submap[0]
        submap_property_list = submap[1]
        for i in range(len(submap_object_list)):
            map_x = center_x + submap_property_list[i]["delta_x"]
            map_y = center_y + submap_property_list[i]["delta_y"]
            # Set value of food layer, specifically for each entry in submap
            if submap_object_list[i] == MapLayer.FOOD:
                self.map[MapLayer.FOOD, map_x, map_y] = submap_property_list[i]["val"]
            # Update position of robot in map
            elif submap_object_list[i] == MapLayer.ROBOT:
                self.map[MapLayer.ROBOT, center_x, center_y] = submap_property_list[i]["id"] + 1 # Set new robot position
                self.map[MapLayer.ROBOT, map_x, map_y] = 0 # Remove old robot position
    
    def findHome(self):
        home_x = []
        home_y = []
        for x in range(self.map_shape[0]):
            for y in range(self.map_shape[1]):
                if self.map[MapLayer.HOME, x, y]:
                    home_x.append(x)
                    home_y.append(y)
        if len(home_x) == 1:
            return (home_x[0], home_y[0]), (home_x, home_y)
        elif len(home_x) > 1:
            avg_x = int(sum(home_x) / len(home_x))
            avg_y = int(sum(home_y) / len(home_y))
            return (avg_x, avg_y), (home_x, home_y)
        else:
            return (sys.maxsize, sys.maxsize), (home_x, home_y)

    def findFoodInfo(self):
        food_pos = []
        food_heading = []
        food_cluster = []
        for x in range(self.map_shape[0]):
            for y in range(self.map_shape[1]):
                if self.map[MapLayer.FOOD, x, y] > 0:
                    food_pos.append([x, y])
                    food_heading.append(self.map[MapLayer.FOOD, x, y])
                    if x >= (self.map_shape[0] // 2):
                        cluster_val = 0
                    else:
                        cluster_val = 1
                    food_cluster.append(cluster_val)
        return food_pos, food_heading, food_cluster
