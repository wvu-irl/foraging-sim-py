from submap_utils import *
from actions import *

def deterministicTransitionModel(states, submap, action, constants):
    new_states = states # new_states, to be returned at end, initialized as current states
    new_submap_object_list = []
    new_submap_property_list = []
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    at_home = isAtHome(0, 0, submap)
    if action == Actions.STAY: # Stay
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
    elif action == Actions.MOVE_E: # Move E
        (delta_x, delta_y) = getDeltaFromDirection(Direction.E)
    elif action == Actions.MOVE_NE: # Move NE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NE)
    elif action == Actions.MOVE_N: # Move N
        (delta_x, delta_y) = getDeltaFromDirection(Direction.N)
    elif action == Actions.MOVE_NW: # Move NW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NW)
    elif action == Actions.MOVE_W: # Move W
        (delta_x, delta_y) = getDeltaFromDirection(Direction.W)
    elif action == Actions.MOVE_SW: # Move SW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SW)
    elif action == Actions.MOVE_S: # Move S
        (delta_x, delta_y) = getDeltaFromDirection(Direction.S)
    elif action == Actions.MOVE_SE: # Move SE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SE)
    elif action == Actions.PIVOT_CW: # Pivot CW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        new_states.heading = getNewHeading(states.heading, Rotation.CW)
    elif action == Actions.PIVOT_CCW: # Pivot CCW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        new_states.heading = getNewHeading(states.heading, Rotation.CCW)
    elif action == Actions.GRAB: # Grab food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food == False and states.battery > 1e-3: # Cannot grab if already posessing food or if battery is dead
            if isFoodAtPos(0, 0, submap):
                new_states.has_food = True
                new_states.food_heading = getFoodHeading(0, 0, submap)
                new_submap_object_list.append(MapLayer.FOOD)
                new_submap_property_list.append({"delta_x" : 0, "delta_y" : 0, "val" : 0}) # Remove food from robot's location on map
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food and states.battery > 1e-3: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, increment number of food retrieved, otherwise cannot drop food
                new_states.num_food_retreived = states.num_food_retreived + 1
                new_states.has_food = False
                new_states.food_heading = 0
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    # If robot battery is dead, robot cannot move
    if states.battery <= 1e-3:
        delta_x = 0
        delta_y = 0

    # Update x and y states
    new_x = current_x + delta_x
    new_y = current_y + delta_y

    # Check of new_x and new_y are within map boundaries
    if new_x >= map_shape[0]:
        new_x = map_shape[0] - 1
        new_y = current_y
    elif new_x < 0:
        new_x = 0
        new_y = current_y
    if new_y >= map_shape[1]:
        new_x = current_x
        new_y = map_shape[1] - 1
    elif new_y < 0:
        new_x = current_x
        new_y = 0
    
    # Check if robot is attempting to move into an obstacle or another robot
    if (not isObstacleAtPos(delta_x, delta_y, submap)) and (not isRobotAtPos(delta_x, delta_y, submap)):
        new_states.x = new_x
        new_states.y = new_y
    # If robot has moved, mark old position in map to be removed
    if new_states.x != current_x or new_states.y != current_y:
        new_submap_object_list.append(MapLayer.ROBOT)
        new_submap_property_list.append({"delta_x" : -delta_x, "delta_y" : -delta_y, "id" : states.robot_id})

    # If at home, battery receives charge
    if at_home:
        new_states.battery = 100.0 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if action == Actions.STAY: # Stay, small battery depletion
            new_states.battery = states.battery - 1.0 # TODO: make it easier to swap out this battery model with others
        elif Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 5.0
        elif action == Actions.GRAB or action == Actions.DROP: # Grab or drop action, large battery depletion
            new_states.battery = states.battery - 10.0

        # Battery cannot be depleted below zero
        if new_states.battery < 0.0:
            new_states.battery = 0.0

    # Update time
    new_states.t = states.t + 1

    # Return new states and new submap
    return (new_states, new_submap)
