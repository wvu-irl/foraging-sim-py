from submap_utils import *
from states import *
from actions import *
import numpy as np
import copy

def mdpDirectionalFoodTransitionModelTrue(states, action, constants):
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    num_food = constants["num_food"]
    food_pos = constants["food_pos"]
    food_heading = constants["food_heading"]
    food_cluster = constants["food_cluster"]
    food_map = getFoodMapFromBinary(states.food_state, num_food, food_pos, map_shape)
    at_home = isAtHome(states.x, states.y, home_pos)
    if action == Actions.STAY: # Stay
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        move_action = True
    elif action == Actions.MOVE_E: # Move E
        (delta_x, delta_y) = getDeltaFromDirection(Direction.E)
        move_action = True
    elif action == Actions.MOVE_NE: # Move NE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NE)
        move_action = True
    elif action == Actions.MOVE_N: # Move N
        (delta_x, delta_y) = getDeltaFromDirection(Direction.N)
        move_action = True
    elif action == Actions.MOVE_NW: # Move NW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NW)
        move_action = True
    elif action == Actions.MOVE_W: # Move W
        (delta_x, delta_y) = getDeltaFromDirection(Direction.W)
        move_action = True
    elif action == Actions.MOVE_SW: # Move SW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SW)
        move_action = True
    elif action == Actions.MOVE_S: # Move S
        (delta_x, delta_y) = getDeltaFromDirection(Direction.S)
        move_action = True
    elif action == Actions.MOVE_SE: # Move SE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SE)
        move_action = True
    elif action == Actions.GRAB: # Grab food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        move_action = False
        new_states = [copy.deepcopy(states)]
        new_states_prob = [1.0]
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
                    new_states = [copy.deepcopy(states), copy.deepcopy(states)]
                    new_states_prob = [grab_success_prob, 1.0 - grab_success_prob]
                    new_states[0].has_food = True
                    new_states[1].has_food = False
                    if isinstance(states, SwarmStates):
                        new_states[0].food_cluster = food_cluster[food_index]
                    food_map[states.x, states.y] = 0 # Remove food from robot's location on map
                    new_states[0].food_state = getBinaryFromFoodMap(food_map, num_food, food_pos)
                    new_states[1].food_state = states.food_state
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        move_action = False
        new_states = [copy.deepcopy(states)]
        new_states_prob = [1.0]
        if states.has_food and states.battery > 0: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, drop food
                new_states[0].has_food = False
                if isinstance(states, SwarmStates):
                    new_states[0].food_cluster = -1
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    if move_action:
        new_states = [copy.deepcopy(states)]
        new_states_prob = [1.0]
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
            
        new_states[0].x = new_x
        new_states[0].y = new_y

        new_at_home = isAtHome(new_states[0].x, new_states[0].y, home_pos)
        
        # If at home, battery receives charge
        if new_at_home:
            new_states[0].battery = constants["battery_size"] - 1
        else:
            # If not at home, battery is depleted based on action taken
            if new_states[0].x != current_x or new_states[0].y != current_y: # Move action, moderate battery depletion
                new_states[0].battery = states.battery - 1

            # Battery cannot be depleted below zero
            if new_states[0].battery < 0:
                new_states[0].battery = 0

    # Return new states
    return new_states, new_states_prob


def mdpDirectionalFoodTransitionModel(states, action, constants, model_num = 0):
    current_x = states.x
    current_y = states.y
    map_shape = constants["map_shape"]
    home_pos = constants["home_pos"]
    num_food = constants["num_food"]
    food_pos = constants["food_pos"]
    food_cluster = constants["food_cluster"]
    food_map = getFoodMapFromBinary(states.food_state, num_food, food_pos, map_shape)
    at_home = isAtHome(states.x, states.y, home_pos)
    if action == Actions.STAY: # Stay
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        move_action = True
    elif action == Actions.MOVE_E: # Move E
        (delta_x, delta_y) = getDeltaFromDirection(Direction.E)
        move_action = True
    elif action == Actions.MOVE_NE: # Move NE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NE)
        move_action = True
    elif action == Actions.MOVE_N: # Move N
        (delta_x, delta_y) = getDeltaFromDirection(Direction.N)
        move_action = True
    elif action == Actions.MOVE_NW: # Move NW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NW)
        move_action = True
    elif action == Actions.MOVE_W: # Move W
        (delta_x, delta_y) = getDeltaFromDirection(Direction.W)
        move_action = True
    elif action == Actions.MOVE_SW: # Move SW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SW)
        move_action = True
    elif action == Actions.MOVE_S: # Move S
        (delta_x, delta_y) = getDeltaFromDirection(Direction.S)
        move_action = True
    elif action == Actions.MOVE_SE: # Move SE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SE)
        move_action = True
    elif action == Actions.GRAB: # Grab food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        move_action = False
        new_states = [copy.deepcopy(states)]
        new_states_prob = [1.0]
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
                    if model_num == 0:
                        grab_success_prob = 0.9
                    elif model_num == 1:
                        grab_success_prob = 0.5
                    elif model_num == 2:
                        if food_cluster[food_index] == 0:
                            grab_success_prob = 0.3
                        elif food_cluster[food_index] == 1:
                            grab_success_prob = 0.5
                    elif model_num == 3:
                        if food_cluster[food_index] == 0:
                            grab_success_prob = 0.7
                        elif food_cluster[food_index] == 1:
                            grab_success_prob = 0.1
                    elif model_num == 4:
                        if food_cluster[food_index] == 0:
                            grab_success_prob = 0.5
                        elif food_cluster[food_index] == 1:
                            grab_success_prob = 0.0
                    new_states = [copy.deepcopy(states), copy.deepcopy(states)]
                    new_states_prob = [grab_success_prob, 1.0 - grab_success_prob]
                    new_states[0].has_food = True
                    new_states[1].has_food = False
                    if isinstance(states, SwarmStates):
                        new_states[0].food_cluster = food_cluster[food_index]
                    food_map[states.x, states.y] = 0 # Remove food from robot's location on map
                    new_states[0].food_state = getBinaryFromFoodMap(food_map, num_food, food_pos)
                    new_states[1].food_state = states.food_state
    elif action == Actions.DROP: # Drop food
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        move_action = False
        new_states = [copy.deepcopy(states)]
        new_states_prob = [1.0]
        if states.has_food and states.battery > 0: # Cannot drop food if not already posessing it or if battery is dead
            if at_home: # If at home, drop food
                new_states[0].has_food = False
                if isinstance(states, SwarmStates):
                    new_states[0].food_cluster = -1
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    if move_action:
        new_states = [copy.deepcopy(states)]
        new_states_prob = [1.0]
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
            
        new_states[0].x = new_x
        new_states[0].y = new_y

        new_at_home = isAtHome(new_states[0].x, new_states[0].y, home_pos)
        
        # If at home, battery receives charge
        if new_at_home:
            new_states[0].battery = constants["battery_size"] - 1
        else:
            # If not at home, battery is depleted based on action taken
            if new_states[0].x != current_x or new_states[0].y != current_y: # Move action, moderate battery depletion
                new_states[0].battery = states.battery - 1

            # Battery cannot be depleted below zero
            if new_states[0].battery < 0:
                new_states[0].battery = 0

    # Return new states
    return new_states, new_states_prob

def unknownMapDirectionalFoodTransitionModelTrue(states, submap, action, constants):
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
                    #new_states.food_heading = food_heading
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
    #print("delta x,y = [{0},{1}]".format(delta_x, delta_y))

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
    # If robot has moved, mark old position in map to be removed and perform food pushing update
    if new_states.x != current_x or new_states.y != current_y:
        # Update robot position in map
        new_submap_object_list.append(MapLayer.ROBOT)
        new_submap_property_list.append({"delta_x" : -delta_x, "delta_y" : -delta_y, "id" : constants["id"]})

        ## Check if food is located at new location
        #if isFoodAtPos(delta_x, delta_y, submap):
        #    # Define the robot's probability of pushing food, based on the direction it approaches from
        #    food_push_prob_pmf = np.array([0.5, 1.0, 0.0, 1.0, 0.5, 0.0, 1.0, 0.0], dtype=np.float)
        #    #food_push_prob_pmf = np.array([0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float)
        #    #food_push_prob_pmf = np.ones(8, dtype=np.float)
        #    food_push_prob_pmf = np.roll(food_push_prob_pmf, states.heading - 1)

        #    # Sample from random number generator to check if robot might push food
        #    move_dir = getDirectionFromDelta(delta_x, delta_y)
        #    food_push_prob_val = food_push_prob_pmf[move_dir - 1]
        #    food_push_rng = np.random.default_rng()
        #    if food_push_rng.random() < food_push_prob_val:
        #        # Check if food would get pushed into other food, robot, obstacle, or edge of map
        #        candidate_new_food_delta_x = delta_x * 2
        #        candidate_new_food_delta_y = delta_y * 2
        #        if (not isFoodAtPos(candidate_new_food_delta_x, candidate_new_food_delta_y, submap)) and (not isRobotAtPos(candidate_new_food_delta_x, candidate_new_food_delta_y, submap)) and (not isObstacleAtPos(candidate_new_food_delta_x, candidate_new_food_delta_y, submap)) and (not isOutsideMap(current_x + candidate_new_food_delta_x, current_y + candidate_new_food_delta_y, map_shape)):
        #            # Record the heading of the food being pushed
        #            food_heading = getFoodHeading(delta_x, delta_y, submap)

        #            # Remove food from current location in map and place it at new location in map
        #            new_submap_object_list.append(MapLayer.FOOD)
        #            new_submap_property_list.append({"delta_x" : 0, "delta_y" : 0, "val" : 0}) # Remove food from robot's new location on map (the new submap is relative to the robot's new position)
        #            new_submap_object_list.append(MapLayer.FOOD)
        #            new_submap_property_list.append({"delta_x" : delta_x, "delta_y" : delta_y, "val" : food_heading}) # Place food at new location to which it was pushed (the new submap is relative to the robot's new position)

    # If at home, battery receives charge
    if at_home:
        new_states.battery = constants["battery_size"] - 1
    else:
        # If not at home, battery is depleted based on action taken
        if Actions.MOVE_E <= action <= Actions.MOVE_SE: # Move action, moderate battery depletion
            new_states.battery = states.battery - 1

        # Battery cannot be depleted below zero
        if new_states.battery < 0:
            new_states.battery = 0

    # Update time
    #new_states.t = states.t + 1

    # Return new states and new submap
    new_submap = (new_submap_object_list, new_submap_property_list)
    return (new_states, new_submap)

def isAtHome(x, y, home_pos):
    if x == home_pos[0] and y == home_pos[1]:
        return True
    else:
        return False

