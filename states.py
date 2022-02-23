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
