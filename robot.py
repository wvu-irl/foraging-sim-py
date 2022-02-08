from abc import ABC, abstractmethod
from action_policies import *
from state_estimators import *
from states import *

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
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_nearest_food_found = False

    def chooseAction(self):
        return simpleFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

class SimpleRandomGrabRobot(Robot):
    def __init__(self, initial_values, constants):
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_nearest_food_found = False
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

class SimpleLocalInteractionRandomGrabRobot(Robot):
    def __init__(self, initial_values, constants):
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_nearest_food_found = False
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabLocalInteractionFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)
