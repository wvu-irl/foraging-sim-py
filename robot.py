from abc import ABC, abstractmethod

class Robot(ABC):
    @abstractmethod
    def __init__(self, initial_states):
        pass

    @abstractmethod
    def chooseAction(self):
        pass

    @abstractmethod
    def stateEstimator(self, observation):
        pass
