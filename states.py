import numpy as np

class FullStates:
    def __init__(self): 
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.food_state = 0 # Binary encoding of food in map, 2^num_food
        
        # Extra states
        self.heading = 0

    def localInfluenceData(self):
        return {"has_food" : self.has_food, "battery" : self.battery}

class States:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.food_state = 0 # Binary encoding of food in map, 2^num_food
    
    def localInfluenceData(self):
        return {"has_food" : self.has_food, "battery" : self.battery}

class SwarmFullStates:
    def __init__(self): 
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.food_state = 0 # Binary encoding of food in map, 2^num_food
        self.food_cluster = -1
        
        # Extra states
        self.heading = 0
    
    def localInfluenceData(self):
        return {"has_food" : self.has_food, "battery" : self.battery, "food_cluster" : self.food_cluster}

class SwarmStates:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.food_state = 0 # Binary encoding of food in map, 2^num_food
        self.food_cluster = -1
    
    def localInfluenceData(self):
        return {"has_food" : self.has_food, "battery" : self.battery, "food_cluster" : self.food_cluster}

class UnknownMapFullStates:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.last_food_x = -1
        self.last_food_y = -1
        self.last_approach_dir = -1
        
        # Extra states
        self.heading = 0
    
    def localInfluenceData(self):
        return {"has_food" : self.has_food, "battery" : self.battery, "last_food_x" : self.last_food_x, "last_food_y" : self.last_food_y, "last_approach_dir" : self.last_approach_dir}

class UnknownMapStates:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.has_food = False
        self.battery = 0
        self.last_food_x = -1
        self.last_food_y = -1
        self.last_approach_dir = -1
    
    def localInfluenceData(self):
        return {"has_food" : self.has_food, "battery" : self.battery, "last_food_x" : self.last_food_x, "last_food_y" : self.last_food_y, "last_approach_dir" : self.last_approach_dir}

def enumerateState(state, state_dimensions):
    state_index = np.ravel_multi_index((state.x, state.y, int(state.has_food == True), state.battery, state.food_state), \
            (state_dimensions["x_size"], state_dimensions["y_size"], state_dimensions["has_food_size"], state_dimensions["battery_size"], 2 ** state_dimensions["num_food"]))
    return state_index

def deEnumerateState(state_index, state_dimensions):
    state = States()
    (x, y, has_food, battery, food_state) = np.unravel_index(state_index, \
            (state_dimensions["x_size"], state_dimensions["y_size"], state_dimensions["has_food_size"], state_dimensions["battery_size"], 2 ** state_dimensions["num_food"]))
    state.x = x
    state.y = y
    state.has_food = bool(has_food)
    state.battery = battery
    state.food_state = food_state

    return state
 
def enumerateFullState(state, state_dimensions):
    state_index = np.ravel_multi_index((state.x, state.y, int(state.has_food == True), state.battery, state.food_state, state.heading), \
            (state_dimensions["x_size"], state_dimensions["y_size"], state_dimensions["has_food_size"], state_dimensions["battery_size"], 2 ** state_dimensions["num_food"], state_dimensions["heading_size"]))
    return state_index

def deEnumerateFullState(state_index, state_dimensions):
    state = States()
    (x, y, has_food, battery, food_state, heading) = np.unravel_index(state_index, \
            (state_dimensions["x_size"], state_dimensions["y_size"], state_dimensions["has_food_size"], state_dimensions["battery_size"], 2 ** state_dimensions["num_food"], state_dimensions["heading_size"]))
    state.x = x
    state.y = y
    state.has_food = bool(has_food)
    state.battery = battery
    state.food_state = food_state
    state.heading = heading

    return state
