from abc import ABC, abstractmethod
from action_policies import *
from state_estimators import *
from reward_functions import *
from states import *

class Robot(ABC):
    @abstractmethod
    def __init__(self, initial_values, constants):
        self.constants = constants

    @abstractmethod # TODO: Maybe using these as abstract methods is not the right workflow for this project...
    def chooseAction(self):
        pass

    @abstractmethod
    def stateEstimator(self, observation):
        pass

    @abstractmethod
    def rewardFunction(self, state, action, state_prime):
        pass

class SimpleDeterministicRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_search_dir_chosen = False
        self.fsm_search_pmf = MovePMFs.uniform
        self.fsm_nearest_food_found = False

    def chooseAction(self):
        return simpleFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class SimpleRandomGrabRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_search_dir_chosen = False
        self.fsm_search_pmf = MovePMFs.uniform
        self.fsm_nearest_food_found = False
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class SimpleLocalInteractionRandomGrabRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_search_dir_chosen = False
        self.fsm_search_pmf = MovePMFs.uniform
        self.fsm_nearest_food_found = False
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabLocalInteractionFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class SingleMDPRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = States()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        policy_filepath = initial_values["policy_filepath"]
        self.policy = np.load(policy_filepath)
        self.state_dimensions = {"x_size" : self.map_shape[0], "y_size" : self.map_shape[1], "has_food_size" : 2, "battery_size" : constants["battery_size"], "num_food" : constants["num_food"]}

    def chooseAction(self):
        state_index = enumerateState(self.states, self.state_dimensions)
        print("state_index: {0}".format(state_index))
        action = self.policy[state_index]
        print("action: {0}".format(action))
        return action

    def stateEstimator(self, observation):
        passthroughMDPStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)
