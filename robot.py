from abc import ABC, abstractmethod
from action_policies import *
from state_estimators import *

class Robot(ABC):
    @abstractmethod
    def __init__(self, initial_values):
        pass

    @abstractmethod
    def chooseAction(self):
        pass

    @abstractmethod
    def stateEstimator(self, observation):
        pass

class SimpleDeterministicRobot(Robot):
    def __init__(self, initial_values):
        pass

    self.chooseAction = simpleFSMActionPolicy
    self.stateEstimator = passthroughStateEstimator
