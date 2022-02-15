from submap_utils import *
from actions import *
import numpy as np

def deterministicTransitionModel(states, submap, action, constants):
    new_states = states # new_states, to be returned at end, initialized as current states
    new_submap_object_list = []
    new_submap_property_list = []
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
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
            if states.at_home: # If at home, increment number of food retrieved, otherwise cannot drop food
                new_states.num_food_retrieved = states.num_food_retrieved + 1
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
    # If robot has moved, mark old position in map to be removed and increment distance traversed
    if new_states.x != current_x or new_states.y != current_y:
        new_submap_object_list.append(MapLayer.ROBOT)
        new_submap_property_list.append({"delta_x" : -delta_x, "delta_y" : -delta_y, "id" : states.robot_id})
        new_states.total_distance_traversed = states.total_distance_traversed + 1

    # Update at_home and num_times_home_visited for new states
    new_states.at_home = isAtHome(new_states.x, new_states.y, home_pos)
    if new_states.at_home == True and states.at_home == False:
        new_states.num_times_home_visited = states.num_times_home_visited + 1

    # If at home, battery receives charge
    if states.at_home:
        new_states.battery = 100.0 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 0.5

        # Battery cannot be depleted below zero
        if new_states.battery < 0.0:
            new_states.battery = 0.0

    # Update time
    new_states.t = states.t + 1

    # Return new states and new submap
    new_submap = (new_submap_object_list, new_submap_property_list)
    return (new_states, new_submap)


def directionalFoodTransitionModel1(states, submap, action, constants):
    new_states = states # new_states, to be returned at end, initialized as current states
    new_submap_object_list = []
    new_submap_property_list = []
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
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
                grab_success_possible_probs = np.array([0.0, 0.0, 0.1, 0.5, 0.9, 0.5, 0.1, 0.0, 0.0]) # Grab success probability associated with each robot-food heading diff [-4, -3, -2, -1, 0, 1, 2, 3, 4] 
                food_heading = getFoodHeading(0, 0, submap)
                robot_food_heading_diff = getHeadingDiff(states.heading, food_heading)
                grab_success_prob = grab_success_possible_probs[robot_food_heading_diff + 4]
                rng = np.random.default_rng()
                if rng.random() < grab_success_prob:
                    new_states.has_food = True
                    new_states.food_heading = food_heading
                    new_submap_object_list.append(MapLayer.FOOD)
                    new_submap_property_list.append({"delta_x" : 0, "delta_y" : 0, "val" : 0}) # Remove food from robot's location on map
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food and states.battery > 1e-3: # Cannot drop food if not already posessing it or if battery is dead
            if states.at_home: # If at home, increment number of food retrieved, otherwise cannot drop food
                new_states.num_food_retrieved = states.num_food_retrieved + 1
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
    # If robot has moved, mark old position in map to be removed and increment distance traversed
    if new_states.x != current_x or new_states.y != current_y:
        new_submap_object_list.append(MapLayer.ROBOT)
        new_submap_property_list.append({"delta_x" : -delta_x, "delta_y" : -delta_y, "id" : states.robot_id})
        new_states.total_distance_traversed = states.total_distance_traversed + 1

    # Update at_home and num_times_home_visited for new states
    new_states.at_home = isAtHome(new_states.x, new_states.y, home_pos)
    print("new at home: {0}, old at home: {1}".format(new_states.at_home, states.at_home))
    if new_states.at_home == True and states.at_home == False:
        print("***************IT HAPPENED*************************")
        new_states.num_times_home_visited = states.num_times_home_visited + 1

    # If at home, battery receives charge
    if states.at_home:
        new_states.battery = 100.0 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 0.5

        # Battery cannot be depleted below zero
        if new_states.battery < 0.0:
            new_states.battery = 0.0

    # Update time
    new_states.t = states.t + 1

    # Return new states and new submap
    new_submap = (new_submap_object_list, new_submap_property_list)
    return (new_states, new_submap)

def isAtHome(x, y, home_pos):
    if x == home_pos[0] and y == home_pos[1]:
        return True
    else:
        return False
