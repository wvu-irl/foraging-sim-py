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
        self.init_pos = constants["init_pos"]
        self.fsm_state = FSMState.SELECT_TARGET

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
        self.init_pos = constants["init_pos"]
        self.fsm_state = FSMState.SELECT_TARGET
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class randomSelectRandomGrabRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = SwarmStates()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.init_pos = constants["init_pos"]
        self.fsm_state = FSMState.SELECT_TARGET
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabRandomSelectFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class randomSelectLocalInteractionRandomGrabRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = SwarmStates()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.init_pos = constants["init_pos"]
        self.fsm_state = FSMState.SELECT_TARGET
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return uncertainGrabRandomSelectLocalInteractionFSMActionPolicy(self)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class UnknownMapFSMRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = SwarmStates()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.init_pos = constants["init_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_search_dir_chosen = False
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return searchFSMActionPolicy(self, False)

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class UnknownMapFSMLocalInteractionRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = SwarmStates()
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.init_pos = constants["init_pos"]
        self.fsm_state = FSMState.SEARCH
        self.fsm_failed_grab_attempts = 0
        self.fsm_failed_food_locations = []

    def chooseAction(self):
        return searchFSMActionPolicy(self, True)

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
        self.init_pos = constants["init_pos"]
        policy_filepath = initial_values["policy_filepath"]
        self.policy = np.load(policy_filepath)
        self.state_dimensions = {"x_size" : self.map_shape[0], "y_size" : self.map_shape[1], "has_food_size" : 2, "battery_size" : constants["battery_size"], "num_food" : constants["num_food"]}

    def chooseAction(self):
        state_index = enumerateState(self.states, self.state_dimensions)
        action = self.policy[state_index]
        return action

    def stateEstimator(self, observation):
        passthroughStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)

class SingleMMMDPRobot(Robot):
    def __init__(self, initial_values, constants):
        super().__init__(initial_values, constants)
        self.states = States()
        self.chosen_action = 0
        self.map_shape = constants["map_shape"]
        self.home_pos = constants["home_pos"]
        self.init_pos = constants["init_pos"]
        self.state_dimensions = {"x_size" : self.map_shape[0], "y_size" : self.map_shape[1], "has_food_size" : 2, "battery_size" : constants["battery_size"], "num_food" : constants["num_food"]}
        self.num_actions = constants["num_actions"]
        self.num_models = initial_values["num_models"]
        self.arbitration_type = initial_values["arbitration_type"]
        self.bum = np.ones(self.num_models, dtype=np.float) / float(self.num_models)
        policy_filepaths = initial_values["policy_filepath"]
        v_filepaths = initial_values["v_filepath"]
        q_filepaths = initial_values["q_filepath"]
        self.policy = [None] * self.num_models
        self.V = [None] * self.num_models
        self.Q = [None] * self.num_models
        for i in range(self.num_models):
            self.policy[i] = np.load(policy_filepaths[i])
            self.V[i] = np.load(v_filepaths[i])
            self.Q[i] = np.load(q_filepaths[i])

    def chooseAction(self):
        if self.arbitration_type == 0:
            self.chosen_action = mmMDPMajorityVotingActionPolicy(self)
        elif self.arbitration_type == 1:
            self.chosen_action = mmMDPWeightedMajorityVotingActionPolicy(self)
        elif self.arbitration_type == 2:
            self.chosen_action = mmMDPHighestPreferenceActionPolicy(self)
        elif self.arbitration_type == 3:
            self.chosen_action = mmMDPWeightedHighestPreferenceActionPolicy(self)
        return self.chosen_action

    def stateEstimator(self, observation):
        passthroughMDPBUMStateEstimator(self, observation)

    def rewardFunction(self, state, action, state_prime):
        mdpRewardFunction(state, action, state_prime)
