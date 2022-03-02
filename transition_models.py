from submap_utils import *
from actions import *
import numpy as np
import copy

def deterministicTransitionModel(states, submap, action, constants):
    new_states = copy.deepcopy(states) # new_states, to be returned at end, initialized as current states
    new_submap_object_list = []
    new_submap_property_list = []
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    at_home = isAtHome(states.x, states.y, home_pos)
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
        if states.has_food == False and states.battery > 0: # Cannot grab if already posessing food or if battery is dead
            if isFoodAtPos(0, 0, submap):
                new_states.has_food = True
                new_states.food_heading = getFoodHeading(0, 0, submap)
                new_submap_object_list.append(MapLayer.FOOD)
                new_submap_property_list.append({"delta_x" : 0, "delta_y" : 0, "val" : 0}) # Remove food from robot's location on map
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food and states.battery > 0: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, drop food
                new_states.has_food = False
                new_states.food_heading = 0
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    # If robot battery is dead, robot cannot move
    if states.battery == 0:
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
        new_states.battery = 10 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 1

        # Battery cannot be depleted below zero
        if new_states.battery < 0:
            new_states.battery = 0

    # Update time
    new_states.t = states.t + 1

    # Return new states and new submap
    new_submap = (new_submap_object_list, new_submap_property_list)
    return (new_states, new_submap)


def directionalFoodTransitionModel1(states, submap, action, constants):
    new_states = copy.deepcopy(states) # new_states, to be returned at end, initialized as current states
    new_submap_object_list = []
    new_submap_property_list = []
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    at_home = isAtHome(states.x, states.y, home_pos)
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
        if states.has_food == False and states.battery > 0: # Cannot grab if already posessing food or if battery is dead
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
        if states.has_food and states.battery > 0: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, drop food
                new_states.has_food = False
                new_states.food_heading = 0
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    # If robot battery is dead, robot cannot move
    if states.battery == 0:
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

    # If at home, battery receives charge
    if at_home:
        new_states.battery = 10 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 1

        # Battery cannot be depleted below zero
        if new_states.battery < 0:
            new_states.battery = 0

    # Update time
    new_states.t = states.t + 1

    # Return new states and new submap
    new_submap = (new_submap_object_list, new_submap_property_list)
    return (new_states, new_submap)


def mdpDirectionalFoodTransitionModelTrue(states, action, constants):
    new_states = copy.deepcopy(states) # new_states, to be returned at end, initialized as current states
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    num_food = constants["num_food"]
    food_pos = constants["food_pos"]
    food_heading = constants["food_heading"]
    food_map = getFoodMapFromBinary(states.food_state, num_food, map_shape)
    at_home = isAtHome(states.x, states.y, home_pos)
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
    elif action == Actions.GRAB: # Grab food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food == False and states.battery > 0: # Cannot grab if already posessing food or if battery is dead
            if food_map[states.x, states.y] == 1:
                # Find which food index is the food at the current location, if there is one
                food_index = -1
                for i in range(num_food):
                    if food_pos[i][0] == states.x and food_pos[i][1] == states.y:
                        food_index = i
                        break
                # Check if food was found at the current location
                if food_index > -1:
                    # If food is at the current location found, attempt to pick up food
                    grab_success_possible_probs = np.array([0.0, 0.0, 0.1, 0.5, 0.9, 0.5, 0.1, 0.0, 0.0]) # Grab success probability associated with each robot-food heading diff [-4, -3, -2, -1, 0, 1, 2, 3, 4] 
                    robot_food_heading_diff = getHeadingDiff(states.heading, food_heading[food_index])
                    grab_success_prob = grab_success_possible_probs[robot_food_heading_diff + 4]
                    rng = np.random.default_rng()
                    if rng.random() < grab_success_prob:
                        new_states.has_food = True
                        new_states.food_heading = food_heading[food_index]
                        food_map[states.x, states.y] = 0 # Remove food from robot's location on map
                        new_states.food_state = getBinaryFromFoodMap(food_map, num_food, food_pos)
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food and states.battery > 0: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, drop food
                new_states.has_food = False
                new_states.food_heading = 0
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    # If robot battery is dead, robot cannot move
    if states.battery == 0:
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
    
    # If at home, battery receives charge
    if at_home:
        new_states.battery = 10 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 1

        # Battery cannot be depleted below zero
        if new_states.battery < 0:
            new_states.battery = 0

    # Update time
    new_states.t = states.t + 1

    # Return new states
    return new_states


def mdpDirectionalFoodTransitionModel(states, action, constants):
    new_states = copy.deepcopy(states) # new_states, to be returned at end, initialized as current states
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    num_food = constants["num_food"]
    food_pos = constants["food_pos"]
    food_map = getFoodMapFromBinary(states.food_state, num_food, map_shape)
    at_home = isAtHome(states.x, states.y, home_pos)
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
    elif action == Actions.GRAB: # Grab food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food == False and states.battery > 0: # Cannot grab if already posessing food or if battery is dead
            if food_map[states.x, states.y] == 1:
                # Find which food index is the food at the current location, if there is one
                food_index = -1
                for i in range(num_food):
                    if food_pos[i][0] == states.x and food_pos[i][1] == states.y:
                        food_index = i
                        break
                # Check if food was found at the current location
                if food_index > -1:
                    # If food is at the current location found, attempt to pick up food
                    grab_success_prob = 0.9 # Grab success probability assumed to be fixed
                    rng = np.random.default_rng()
                    if rng.random() < grab_success_prob:
                        new_states.has_food = True
                        food_map[states.x, states.y] = 0 # Remove food from robot's location on map
                        new_states.food_state = getBinaryFromFoodMap(food_map, num_food, food_pos)
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        if states.has_food and states.battery > 0: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, drop food
                new_states.has_food = False
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    # If robot battery is dead, robot cannot move
    if states.battery == 0:
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
    
    # If at home, battery receives charge
    if at_home:
        new_states.battery = 10 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 1

        # Battery cannot be depleted below zero
        if new_states.battery < 0:
            new_states.battery = 0

    # Update time
    new_states.t = states.t + 1

    # Return new states
    return new_states

def mdpDirectionalFoodTransitionModelProb(state, action, state_prime, constants):
    grab_success_prob = 0.9 # Grab success probability assumed to be fixed
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    num_food = constants["num_food"]
    food_pos = constants["food_pos"]
    food_map = getFoodMapFromBinary(states.food_state, num_food, map_shape)
    if action == Actions.STAY:
        if state.x == state_prime.x and state.y == state_prime.y and state.has_food == state_prime.has_food \\
                and state.battery == state_prime.battery and state.food_state == state_prime.food_state:
            prob = 1.0
        else:
            prob = 0.0
    elif Actions.MOVE_E <= action <= Actions.MOVE_SE:
        if state.battery == 0: # If robot battery is dead, robot cannot move
            delta_x = 0
            delta_y = 0
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

        current_x = state.x
        current_y = state.y

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

        new_battery = state.battery - 1
        if new_battery < 0:
            new_battery = 0

        if isAtHome(state_prime.x, state_prime.y, home_pos):
            new_battery = 10

        if new_x == state_prime.x and new_y == state_prime.y and new_battery == state_prime.battery \\
                and state.has_food == state_prime.has_food and state.food_state == state_prime.food_state:
            prob = 1.0
        else:
            prob = 0.0
    elif action == Actions.GRAB:
        prob = 0.0
        if state.has_food == False and state.battery > 0 and state.x == state_prime.x and state.y == state_prime.y: # Cannot grab if already posessing food or if battery is dead
            if food_map[state.x, state.y] == 1:
                # Find which food index is the food at the current location, if there is one
                food_index = -1
                for i in range(num_food):
                    if food_pos[i][0] == state.x and food_pos[i][1] == state.y:
                        food_index = i
                        break
                # Check if food was found at the current location
                if food_index > -1:
                    food_map[state.x, state.y] = 0 # Remove food from robot's location on map
                    new_food_state = getBinaryFromFoodMap(food_map, num_food, food_pos)
                    if state_prime.has_food == True and new_food_state == state_prime.food_state:
                        prob = grab_success_prob
                    elif state_prime.has_food == False and food_state == state_prime.food_state:
                        prob = 1.0 - grab_success_prob
    elif action == Actions.DROP:
        prob = 0.0
        if state.has_food and states.battery > 0 and state.x == state_prime.x and state.y == state_prime.y and state.food_state == state_prime.food_state: # Cannot drop food if not already posessing it or if battery is dead
            if isAtHome(state.x, state.y, home_pos) and state_prime.has_food == False:
                prob = 1.0
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    return prob


def isAtHome(x, y, home_pos):
    if x == home_pos[0] and y == home_pos[1]:
        return True
    else:
        return False


def getFoodMapFromBinary(food_state, num_food, map_shape):
    food_map = np.zeros(map_shape)
    for i in range(num_food):
        food_present = (food_state >> i) & 1
        food_map[food_pos[i][0], food_pos[i][1]] = food_present
    return food_map


def getBinaryFromFoodMap(food_map, num_food, food_pos):
    food_state = 0
    map_shape = food_pos.shape()
    for x in range(map_shape[0]):
        for y in range(map_shape[1]):
            if food_map[x, y] == 1:
                for i in range(num_food):
                    if x == food_pos[i][0] and y == food_pos[i][1]:
                        food_state += (1 << i)
    return food_state
