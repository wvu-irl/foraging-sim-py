from foraging_map import *
from enum import IntEnum, unique
import sys
from debug_print import debugPrint

@unique
class Direction(IntEnum):
    NONE = 0
    E = 1
    NE = 2
    N = 3
    NW = 4
    W = 5
    SW = 6
    S = 7
    SE = 8

@unique
class Rotation(IntEnum):
    NONE = 0
    CW = 1
    CCW = 2

def getDeltaFromDirection(direc): # returns (delta_x, delta_y)
    if direc == Direction.E:
        return (1, 0)
    elif direc == Direction.NE:
        return (1, 1)
    elif direc == Direction.N:
        return (0, 1)
    elif direc == Direction.NW:
        return (-1, 1)
    elif direc == Direction.W:
        return (-1, 0)
    elif direc == Direction.SW:
        return (-1, -1)
    elif direc == Direction.S:
        return (0, -1)
    elif direc == Direction.SE:
        return (1, -1)
    elif direc == Direction.NONE:
        return (0, 0)
    else:
        raise RuntimeError("getDeltaFromDirection: not a valid direction value: {0}".format(direc))

def getNewHeading(current_heading, rotation):
    if rotation == Rotation.CW:
        new_heading = current_heading + 1
    elif rotation == Rotation.CCW:
        new_heading = current_heading - 1

    if new_heading < 1:
        new_heading = 8
    elif new_heading > 8:
        new_heading = 1

    return new_heading

def getHeadingDiff(heading1, heading2):
    diff = heading1 - heading2
    if diff > 4:
        diff -= 8
    elif diff < -4:
        diff += 8

    return diff

def isFoodAtPos(delta_x, delta_y, submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a food entry
        if submap_object_list[i] == MapLayer.FOOD:
            # Check if food delta position matches query
            if delta_x == submap_property_list[i]["delta_x"] and delta_y == submap_property_list[i]["delta_y"]:
                return True
    
    # If code falls through to here, then there is no food at the query position
    return False

def isFoodVisible(submap, exclude_locations = [], robot_x = sys.maxsize, robot_y = sys.maxsize):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a food entry
        if submap_object_list[i] == MapLayer.FOOD:
            # Exclude locations marked as excluded food
            exclude = False
            for j in range(len(exclude_locations)):
                if submap_property_list[i]["delta_x"] == (exclude_locations[j]["x"] - robot_x) and submap_property_list[i]["delta_y"] == (exclude_locations[j]["y"] - robot_y):
                    debugPrint("isFoodVisible: EXCLUDE FAILED FOOD")
                    exclude = True
            if exclude == False:
                return True
    # If code falls through to here, then there is no food or no non-excluded food in the submap
    return False

def findNearestFood(submap, exclude_locations = [], robot_x = sys.maxsize, robot_y = sys.maxsize):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    nearest_distance = sys.maxsize
    nearest_delta_x = sys.maxsize
    nearest_delta_y = sys.maxsize
    other_robot_delta_x = []
    other_robot_delta_y = []
    
    for i in range(len(submap_object_list)):
        # Check iif entry is a robot entry
        if submap_object_list[i] == MapLayer.ROBOT:
            # Record its location
            other_robot_delta_x.append(submap_property_list[i]["delta_x"])
            other_robot_delta_y.append(submap_property_list[i]["delta_y"])

    for i in range(len(submap_object_list)):
        # Check if entry is a food entry
        if submap_object_list[i] == MapLayer.FOOD:
            # Check if food distance is closer than the previously closest found food and if location is not on the exclude list
            distance = max(abs(submap_property_list[i]["delta_x"]), submap_property_list[i]["delta_y"])
            # Exclude locations marked as excluded food or occupied by other robot
            exclude = False
            for j in range(len(exclude_locations)):
                if submap_property_list[i]["delta_x"] == (exclude_locations[j]["x"] - robot_x) and submap_property_list[i]["delta_y"] == (exclude_locations[j]["y"] - robot_y):
                    debugPrint("findNearestFood: EXCLUDE FAILED FOOD")
                    exclude = True

            for k in range(len(other_robot_delta_x)):
                if submap_property_list[i]["delta_x"] == other_robot_delta_x[k] and submap_property_list[i]["delta_y"] == other_robot_delta_y[k]:
                    debugPrint("findNearestFood: EXCLUDE FOOD OCCUPIED BY OTHER ROBOT")
                    exclude = True
            
            debugPrint("food distance: {0}".format(distance))
            debugPrint("food delta_x, delta_y: [{0},{1}]".format(submap_property_list[i]["delta_x"],submap_property_list[i]["delta_y"]))
            if distance < nearest_distance and not exclude:
                nearest_distance = distance
                nearest_delta_x = submap_property_list[i]["delta_x"]
                nearest_delta_y = submap_property_list[i]["delta_y"]

    return (nearest_delta_x, nearest_delta_y)

def getFoodHeading(delta_x, delta_y, submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a food entry
        if submap_object_list[i] == MapLayer.FOOD:
            # Check if food delta position matches query
            if delta_x == submap_property_list[i]["delta_x"] and delta_y == submap_property_list[i]["delta_y"]:
                # If so, return the heading of the food
                return submap_property_list[i]["heading"]
    
    # If code falls through to here, then there is no food at the query position
    return 0

def isAtHome(submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a home entry
        if submap_object_list[i] == MapLayer.HOME:
            # Check if agent is at home
            if submap_property_list[i]["delta_x"] == 0 and submap_property_list[i]["delta_y"] == 0:
                return True
    
    # If code falls through to here, then there is no food at the query position
    return False

def isObstacleAtPos(delta_x, delta_y, submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is an obstacle entry
        if submap_object_list[i] == MapLayer.OBSTACLE:
            # Check if food delta position matches query
            if delta_x == submap_property_list[i]["delta_x"] and delta_y == submap_property_list[i]["delta_y"]:
                return True

def isRobotAtPos(delta_x, delta_y, submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a robot entry
        if submap_object_list[i] == MapLayer.ROBOT:
            # Check if food delta position matches query
            if delta_x == submap_property_list[i]["delta_x"] and delta_y == submap_property_list[i]["delta_y"]:
                return True

def atEdgeOfMap(x, y, map_shape):
    x_at_edge = (x == map_shape[0] - 1) or (x == 0)
    y_at_edge = (y == map_shape[1] - 1) or (y == 0)
    if x_at_edge or y_at_edge:
        return True
    else:
        return False

