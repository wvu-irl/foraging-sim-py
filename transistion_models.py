from submap_utils import *

def deterministicTransitionModel(states, submap, action):
    new_states = states # new_states, to be returned at end, initialized as current states
    new_submap_object_list = []
    new_submap_property_list = []
    current_x = states.x
    current_y = states.y
    if action == 0: # Stay
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
    elif action == 1: # Move E
        (delta_x, delta_y) = getDeltaFromDirection(Direction.E)
    elif action == 2: # Move NE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NE)
    elif action == 3: # Move N
        (delta_x, delta_y) = getDeltaFromDirection(Direction.N)
    elif action == 4: # Move NW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NW)
    elif action == 5: # Move W
        (delta_x, delta_y) = getDeltaFromDirection(Direction.W)
    elif action == 6: # Move SW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SW)
    elif action == 7: # Move S
        (delta_x, delta_y) = getDeltaFromDirection(Direction.S)
    elif action == 8: # Move SE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.SE)
    elif action == 9: # Grab food E
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.E)
    elif action == 10: # Grab food NE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.NE)
    elif action == 11: # Grab food N
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.N)
    elif action == 12: # Grab food NW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.NW)
    elif action == 13: # Grab food W
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.W)
    elif action == 14: # Grab food SW
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.SW)
    elif action == 15: # Grab food S
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.S)
    elif action == 16: # Grab food SE
        (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
        (food_delta_x, food_delta_y) = getDeltaFromDirection(Direction.SE)
    else:
        raise RuntimeError("action is not valid: {0}".format(action))

    # Update x and y states
    new_x = current_x + delta_x
    new_y = current_y + delta_y
    new_states.x = new_x
    new_states.y = new_y # TODO: add checks for moving off edge of map or into obstacles or other robots, and dead battery
    # If robot has moved, mark old position in map to be removed
    if new_x != current_x and new_y != current_y:
        new_submap_object_list.append(MapLayer.ROBOT)
        new_submap_property_list.append({"delta_x" : -delta_x, "delta_y" : -delta_y, "id" : states.id})

    # If a grab action was taken, perform the grab update
    if 9 <= action <= 16: # range of grab actions 
        if isFoodAtPos(food_delta_x, food_delta_y, submap):
            new_states.has_food = True
            new_submap_object_list.append(MapLayer.FOOD)
            new_submap_property_list.append({"delta_x" : food_delta_x, "delta_y" : food_delta_y, "val" : 0})

    # Check if on a home grid
    if isAtHome(submap):
        # If at home and has_food, food gets dropped
        if states.has_food:
            new_states.has_food = False

        # If at home, battery receives charge
        new_states.battery = 100.0 # TODO: have ability to switch out more sophisticated battery charging model
    else:
        # If not at home, battery is depleted based on action taken
        if action == 0: # Stay, small battery depletion
            new_states.battery = states.battery - 1.0 # TODO: make it easier to swap out this battery model with others
        elif 1 <= action <= 8: # Move action, moderate battery depletion
            new_states.battery = states.battery - 5.0
        elif 9 <= action <= 16: # Grab action, large battery depletion
            new_states.battery = states.battery - 10.0

        # Battery cannot be depleted below zero
        if new_states.battery < 0.0:
            new_states.battery = 0.0

    # Return new states and new submap
    return (new_states, new_submap)
