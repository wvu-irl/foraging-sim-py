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

def getDirectionFromDelta(delta_x, delta_y): # returns Direction
    delta = (delta_x, delta_y)
    if delta == (1, 0):
        return Direction.E
    elif delta == (1, 1):
        return Direction.NE
    elif delta == (0, 1):
        return Direction.N
    elif delta == (-1, 1):
        return Direction.NW
    elif delta == (-1, 0):
        return Direction.W
    elif delta == (-1, -1):
        return Direction.SW
    elif delta == (0, -1):
        return Direction.S
    elif delta == (1, -1):
        return Direction.SE
    elif delta == (0, 0):
        return Direction.NONE
    else:
        raise RuntimeError("getDirectionFromDelta: not a valid [delta_x, delta_y]: [{0}, {1}]".format(delta_x, delta_y))

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
            distance = max(abs(submap_property_list[i]["delta_x"]), abs(submap_property_list[i]["delta_y"])) # Chebyshev distance
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

def isRobotVisible(submap, personality = -1):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a robot entry
        if submap_object_list[i] == MapLayer.ROBOT:
            # If a robot personality type is requested, check if it is that personality. Otherwise, any robot is acceptable.
            if submap_property_list[i]["personality"] == personality or personality == -1:
                return True

    # If code falls through to here, then there is no robot visible
    return False

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

    # If code falls through to here, then there is no robot at the query position
    return False

def findNearestRobot(submap, personality = -1):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    nearest_distance = sys.maxsize
    nearest_delta_x = sys.maxsize
    nearest_delta_y = sys.maxsize
    nearest_has_food = False

    for i in range(len(submap_object_list)):
        # Check if entry is a robot entry
        if submap_object_list[i] == MapLayer.ROBOT:
            # If a robot personality type is requested, check if it is that personality. Otherwise, any robot is acceptable.
            if submap_property_list[i]["personality"] == personality or personality == -1:
                # Check if robot distance is closer than the previously closest found robot
                distance = max(abs(submap_property_list[i]["delta_x"]), abs(submap_property_list[i]["delta_y"])) # Chebyshev distance
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_delta_x = submap_property_list[i]["delta_x"]
                    nearest_delta_y = submap_property_list[i]["delta_y"]
                    nearest_has_food = submap_property_list[i]["has_food"]

    return (nearest_delta_x, nearest_delta_y, nearest_has_food)

def listVisibleRobotProperties(submap, personality = -1):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    robot_properties = []
    for i in range(len(submap_object_list)):
        # Check if entry is a robot entry
        if submap_object_list[i] == MapLayer.ROBOT:
            # If a robot personality type is requested, check if it is that personality. Otherwise, any robot is acceptable.
            if submap_property_list[i]["personality"] == personality or personality == -1:
                robot_properties.append({"has_food" : submap_property_list[i]["has_food"], "food_cluster" : submap_property_list[i]["food_cluster"]})

    return robot_properties

def findBlockedMoves(submap):
    blocked_moves = []
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a robot or obstacle entry
        if submap_object_list[i] == MapLayer.ROBOT or submap_object_list[i] == MapLayer.OBSTACLE:
            # Check if distance is 1, meaning only 1 move away, potentially blocking a move
            distance = max(abs(submap_property_list[i]["delta_x"]), abs(submap_property_list[i]["delta_y"])) # Chebyshev distance
            if distance == 1:
                # If so, add this direction to the blocked moves list
                blocked_moves.append(getDirectionFromDelta(submap_property_list[i]["delta_x"], submap_property_list[i]["delta_y"]))
    
    return blocked_moves


def atEdgeOfMap(x, y, map_shape):
    x_at_edge = (x == map_shape[0] - 1) or (x == 0)
    y_at_edge = (y == map_shape[1] - 1) or (y == 0)
    if x_at_edge or y_at_edge:
        return True
    else:
        return False

def getFoodMapFromBinary(food_state, num_food, food_pos, map_shape):
    food_map = np.zeros(map_shape)
    for i in range(num_food):
        food_present = (food_state >> i) & 1
        food_map[food_pos[i][0], food_pos[i][1]] = food_present
    return food_map


def getBinaryFromFoodMap(food_map, num_food, food_pos):
    food_state = 0
    map_shape = food_map.shape
    for x in range(map_shape[0]):
        for y in range(map_shape[1]):
            if food_map[x, y] >= 1:
                for i in range(num_food):
                    if x == food_pos[i][0] and y == food_pos[i][1]:
                        food_state += (1 << i)
    return food_state
