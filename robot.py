from abc import ABC, abstractmethod
from states import States

class Robot(ABC):
    def __init__(self, x_init, y_init, t_init, local_map_init, battery_init, personality_init):
        self.states = States
        self.states.x = x_init
        self.states.y = y_init
        self.states.t = t_init
        self.states.has_food = False
        self.states.battery = battery_init
        self.states.personality = personality_init
        self.local_map = local_map_init
        self.neighbors = []
        self.next_action = 0
        super().__init__()

    @abstractmethod
    def chooseAction(self):
        pass

    @abstractmethod
    def transitionModel(self, action):
        pass

    @abstractmethod
    def stateEstimator(self, observation):
