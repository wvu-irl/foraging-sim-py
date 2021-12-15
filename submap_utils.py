from foraging_map import *

@unique
class Direction(Enum):
    E = 0
    NE = 1
    N = 2
    NW = 3
    W = 4
    SW = 5
    S = 6
    SE = 7
    NONE = 8

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

def isFoodAtPos(delta_x, delta_y, submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a food entry
        if submap_object_list[i] == MapLayer.FOOD:
            # Check if food delta position matches query
            if delta_x == submap_property_list[i]{"delta_x"} and delta_y == submap_property_list[i]{"delta_y"}:
                return True
    
    # If code falls through to here, then there is no food at the query position
    return False

def isAtHome(submap):
    # Loop over entries in submap list
    submap_object_list = submap[0]
    submap_property_list = submap[1]
    for i in range(len(submap_object_list)):
        # Check if entry is a home entry
        if submap_object_list[i] == MapLayer.HOME:
            # Check if agent is at home
            if submap_property_list[i]{"delta_x"} == 0 and submap_property_list[i]{"delta_y"} == 0:
                return True
    
    # If code falls through to here, then there is no food at the query position
    return False
