class FullStates:
    def __init__(self): 
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.food_state = 0 # Binary encoding of food in map, 2^num_food
        
        # Extra states
        self.heading = 0
        self.t = 0
        self.food_heading = 0

class States:
    def __init(self):
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.food_state = 0 # Binary encoding of food in map, 2^num_food

def enumerateState(state, state_max_vals):
    state_index = 0
    state_index += state.x
    state_index += state.y * state_max_vals["max_x"]
    state_index += int(state.has_food == True) * state_max_vals["max_x"] * state_max_vals["max_y"]
    state_index += state.battery * state_max_vals["max_x"] * state_max_vals["max_y"] * 2
    state_index += state.food_state * state_max_vals["max_x"] * state_max_vals["max_y"] * 2 * state_max_vals["max_battery"]

    return state_index
