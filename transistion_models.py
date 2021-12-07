def deterministicTransitionModel(states, action):
    new_states = states # new_states, to be returned at ent, initialized as current states
    current_x = states.x
    current_y = states.y
    if action == 0: # Stay
        new_x = current_x
        new_y = current_y
    elif action == 1: # Move E
        new_x = current_x + 1
        new_y = current_y
    elif action == 2: # Move NE
        new_x = current_x + 1
        new_y = current_y + 1
    elif action == 3: # Move N
        new_x = current_x
        new_y = current_y + 1
    elif action == 4: # Move NW
        new_x = current_x - 1
        new_y = current_y + 1
    elif action == 5: # Move W
        new_x = current_x - 1
        new_y = current_y
    elif action == 6: # Move SW
        new_x = current_x - 1
        new_y = current_y - 1
    elif action == 7: # Move S
        new_x = current_x
        new_y = current_y - 1
    elif action == 8: # Move SE
        new_x = current_x + 1
        new_y = current_y - 1
    elif action == 9: # Grab food
        new_x = current_x
        new_y = current_y


