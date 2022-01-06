from abc import ABC, abstractmethod
from action_policies import *
from state_estimators import *

class Robot(ABC):
    @abstractmethod
    def __init__(self, initial_values, constants):
        pass

    @abstractmethod # TODO: Maybe using these as abstract methods is not the right workflow for this project...
    def chooseAction(self):
        pass

    @abstractmethod
    def stateEstimator(self, observation):
        pass

class SimpleDeterministicRobot(Robot):
    def __init__(self, initial_values, constants):
        self.map_shape = constants["map_shape"]
        self.chooseAction = simpleFSMActionPolicy # TODO: See above. This is circumventing these as abstract methods, anyway.
        self.stateEstimator = passthroughStateEstimator
        self.fsm_state = FSMState.SEARCH
        self.fsm_nearest_food_found = False

    def chooseAction(self):
        pass

    def stateEstimator(self):
        pass
