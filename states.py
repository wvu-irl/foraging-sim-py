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

def enumerateState(state, state_dimensions):
    state_index = np.ravel_multi_index((state.x, state.y, int(state.has_food == True), state.battery, state.food_state), \
            (state_dimensions["x_size"], state_dimensions["y_size"], 2, state_dimensions["max_battery"], 2 ** state_dimensions["num_food"]))
    return state_index

def deEnumerateState(state_index, state_dimensions):
    state = States()
    (x, y, has_food, battery, food_state) = np.unravel_index(state_index, \
            (state_dimensions["x_size"], state_dimensions["y_size"], 2, state_dimensions["max_battery"], 2 ** state_dimensions["num_food"]))
    state.x = x
    state.y = y
    state.has_food = bool(has_food)
    state.battery = battery
    state.food_state = food_state

    return state
 
